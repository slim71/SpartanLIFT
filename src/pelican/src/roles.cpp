#include "pelican.hpp"
#include "logger.hpp"
#include "types.hpp"
#include "utilities.hpp"

void Pelican::becomeLeader() {
    this->setRole(leader);
    this->logger_.logInfo("Becoming {}", main_module, roles_to_string(this->getRole()));
    this->hb_core_.flushStorage();

    // Unsubscribe from topics
    this->hb_core_.resetSubscription();
    // This is not needed by leaders, since Raft guarantees safety;
    // it's just an additional check to avoid multiple leaders
    resetSharedPointer(this->sub_to_leader_election_topic_); // TODO: define a public access
    resetSharedPointer(this->sub_to_request_vote_rpc_topic_); // TODO: define a public access

    this->hb_core_.setupPublisher();

    this->hb_core_.sendNow(); // To promptly notify all agents about the new leader
    this->hb_core_.setTransmissionTimer();
}

void Pelican::becomeFollower() {
    this->setRole(follower);
    this->logger_.logInfo("Becoming {}", main_module, roles_to_string(this->getRole()));
    this->prepareCommonCallbacks();
    this->setRandomElectionTimeout();

    this->sub_to_request_vote_rpc_topic_ = this->create_subscription<comms::msg::RequestVoteRPC>(
        this->request_vote_rpc_topic_, this->standard_qos_,
        std::bind(&Pelican::serveVoteRequest, this, std::placeholders::_1), this->reentrant_opt_
    );

    this->election_timer_ = this->create_wall_timer(
        this->election_timeout_,
        [this]() {
            cancelTimer(this->election_timer_);
            if (this->getRole() == follower) {
                this->logger_.logWarning("No heartbeat received within the 'election_timeout' window; "
                                 "switching to candidate...", main_module);
                this->becomeCandidate(); // transition to Candidate state
            }
        },
        this->reentrant_group_
    );
}

void Pelican::becomeCandidate() {
    this->setRole(candidate);
    this->logger_.logInfo("Becoming {}", main_module, roles_to_string(this->getRole()));
    this->prepareCommonCallbacks();
    cancelTimer(this->election_timer_);
    resetSharedPointer(this->sub_to_request_vote_rpc_topic_); // unsubscribe from topic

    this->clearElectionStatus();
    this->unsetVotingCompleted();
    this->unsetElectionCompleted();
    // init of new_ballot_waittime_
    this->setRandomBallotWaittime();

    // the candidate repeats this until:
    // (a) it wins the election                         --> election_completed_ = true
    // (b) another server establishes itself as leader  --> election_completed_ = false +
    // external_leader_elected = true (?) (c) a period of time goes by with no winner      -->
    // nothing signaling this, just restart from a newer term_id
    while (!this->checkElectionCompleted()) {
        // For the first execution the init phase has been done before the while loop

        this->flushVotes();

        this->logger_.logDebug("Going to sleep for {} ms...", main_module, this->getBallotWaitTime().count());
        std::this_thread::sleep_for(this->getBallotWaitTime());

        this->increaseCurrentTerm();
        this->logger_.logInfo("New voting round for term {}", main_module, this->getCurrentTerm());

        this->vote(this->getID(), this->getMass());
        this->requestVote();

        this->startBallotThread();

        // Wait until voting is completed (aka no more votes are registered in a time frame) or
        // timed-out, which is checked by another thread
        std::unique_lock<std::mutex> lock(this->candidate_mutex_);
        this->cv.wait(lock, [this]() {
            return (this->checkVotingCompleted() || this->checkForExternalLeader());
        });
        this->logger_.logDebug("Main thread free from ballot lock", main_module);

        if (this->checkForExternalLeader()) {
            this->logger_.logInfo("External leader elected", main_module);
            this->setElectionCompleted();
            this->becomeFollower();
            continue; // or break, should be the same since the condition is set
        }

        if (this->checkVotingCompleted()) {
            this->logger_.logInfo(
                "Voting completed for term {}. Checking if I'm the winner...", main_module,
                this->getCurrentTerm()
            );
            this->leaderElection(
            ); // if this node is not the leader, election_completed_ is not true
        }

        // No other possible reasons to exit the condition_variable wait
    };
}

void Pelican::commenceFollowerOperations() {
    this->becomeFollower();
}

bool Pelican::isLeader() const {
    this->logger_.logDebug("Agent is leader? {}", main_module, this->getRole() == leader);
    return (this->getRole() == leader);
}

bool Pelican::isFollower() const {
    this->logger_.logDebug("Agent is follower? {}", main_module, this->getRole() == follower);
    return (this->getRole() == follower);
}

bool Pelican::isCandidate() const {
    this->logger_.logDebug("Agent is candidate? {}", main_module, this->getRole() == candidate);
    return (this->getRole() == candidate);
}
