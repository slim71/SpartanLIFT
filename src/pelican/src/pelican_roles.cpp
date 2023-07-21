#include "pelican.hpp"

void PelicanUnit::becomeLeader() {
    this->logInfo("Becoming Leader");
    this->setRole(leader);

    // Unsubscribe from the heartbeat topic // TODO: not to do, in case of another leader arriving?
    this->sub_to_heartbeat_topic_.reset();

    this->pub_to_heartbeat_topic_ = this->create_publisher<comms::msg::Heartbeat>(
                                                this->heartbeat_topic_,
                                                this->standard_qos_
                                            );

    this->hb_transmission_timer_ = this->create_wall_timer(this->heartbeat_period_, 
                                                           std::bind(&PelicanUnit::sendHeartbeat, this), 
                                                           this->reentrant_group_);
}

void PelicanUnit::becomeFollower() {
    this->logInfo("Becoming Follower");
    this->setRole(follower);

    // TODO: needed by all agents? maybe, to let candidates and old leaders know they have to step down
    this->sub_to_request_vote_rpc_topic_ = this->create_subscription<comms::msg::RequestVoteRPC>(
                                        this->request_vote_rpc_topic_,
                                        this->standard_qos_, 
                                        std::bind(&PelicanUnit::serveVoteRequest, this, std::placeholders::_1),
                                        this->reentrant_opt_
                                    );

    this->setRandomElectionTimeout();

    this->election_timer_ = this->create_wall_timer(this->election_timeout_, 
                                                    std::bind(&PelicanUnit::becomeCandidate, this), 
                                                    this->reentrant_group_
                                                   );
}

void PelicanUnit::becomeCandidate() {
    this->logInfo("Becoming Candidate");
    this->election_timer_->cancel();
    this->setRole(candidate);

    this->unsetLeaderElected();
    this->unsetVotingCompleted();
    this->unsetElectionCompleted();
    // init of new_ballot_waittime_
    this->setRandomBallotWaittime();

    this->sub_to_request_vote_rpc_topic_.reset(); // unsubscribe from topic

    // the candidate repeats this until:
    // (a) it wins the election                         --> election_completed_ = true
    // (b) another server establishes itself as leader  --> election_completed_ = false + external_leader_elected = true (?)
    // (c) a period of time goes by with no winner      --> nothing signaling this, just restart from a newer term_id
    while(!this->checkElectionCompleted()) {
        // For the first execution the init phase has been done before the while loop

        this->flushVotes();

        std::this_thread::sleep_for(this->getBallotWaitTime());

        this->increaseCurrentTerm();

        this->vote(this->getID(), this->getMass());
        this->requestVote();

        this->startBallotThread();

        // Wait until voting is completed (aka no more votes are registered in a time frame) or timed-out, which is checked by another thread
        std::unique_lock<std::mutex> lock(this->candidate_mutex_);
        cv.wait(lock, [this](){ 
            return (this->checkVotingCompleted() || this->checkForExternalLeader());
        });

        if(this->checkForExternalLeader()) {
            this->logInfo("External leader elected");
            this->setElectionCompleted();
            this->becomeFollower();
            continue; // or break, should be the same since the condition is set
        }

        if(this->checkVotingCompleted()) {
            this->logInfo("Voting completed. Checking if I'm the winner...");
            this->leaderElection(); // if this node is not the leader, election_completed_ is not true
        }

        // No other possible reasons to exit the condition_variable wait
    };

}

bool PelicanUnit::isLeader() {
    this->logDebug("Agent is leader? {}", this->getRole() == leader);
    return (this->getRole() == leader);
}

bool PelicanUnit::isFollower() {
    this->logDebug("Agent is follower? {}", this->getRole() == follower);
    return (this->getRole() == follower);
}

bool PelicanUnit::isCandidate() {
    this->logDebug("Agent is candidate? {}", this->getRole() == candidate);
    return (this->getRole() == candidate);
}
