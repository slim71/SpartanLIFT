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

    // this->hb_monitoring_timer_ = this->create_wall_timer(this->hb_monitoring_period_, 
    //                                                      std::bind(&PelicanUnit::checkHeartbeat, this), 
    //                                                      this->reentrant_group_
    //                                                     );

    this->setRandomElectionTimeout();

    this->election_timer_ = this->create_wall_timer(this->election_timeout_, 
                                                    std::bind(&PelicanUnit::setElectionTimedOut, this), 
                                                    this->reentrant_group_
                                                   );
}

void PelicanUnit::becomeCandidate() {
    this->logInfo("Becoming Candidate");
    this->setRole(candidate);

    this->unsetLeaderElected();
    this->unsetVotingCompleted();
    this->unsetElectionCompleted();

    this->sub_to_request_vote_rpc_topic_.reset(); // unsubscribe from topic

    // init of new_ballot_waittime_
    this->setRandomBallotWaittime();

    // the candidate repeats this until:
    // (a) it wins the election                         --> election_completed_ = true
    // (b) another server establishes itself as leader  --> election_completed_ = false + external_leader_elected = true (?)
    // (c) a period of time goes by with no winner      --> nothing signaling this, just restart from a newer term_id
    while(!this->checkElectionCompleted()) {
        // For the first execution the init phase has been done before the while loop

        this->flushVotes();

        std::this_thread::sleep_for(this->new_ballot_waittime_);

        this->increaseCurrentTerm();

        this->vote(this->getID());
        this->requestVote();

        std::thread worker([this]() { this->ballotCheckingThread(); });

        // Wait until voting is completed (aka no more votes are registered in a time frame) or timed-out, which is checked by another thread
        std::unique_lock<std::mutex> lock(this->candidate_mutex_);
        cv.wait(lock, [this](){ 
            return (this->checkVotingCompleted() || this->checkForExternalLeader());
        });

        if(this->checkForExternalLeader()) {
            this->setElectionCompleted();
            this->becomeFollower();
        }

        if(this->checkVotingCompleted()) {
            this->leaderElection(); // if this node is not the leader, election_completed_ is not true
        }

        // No other possible reasons to exit the condition_variable wait
    };

}

bool PelicanUnit::isLeader() {
    this->logDebug("Agent is leader? {}", this->getRole() == leader);
    return (this->getRole() == leader);
}
