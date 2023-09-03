#include "election.hpp"
#include "pelican.hpp"
#include "types.hpp"

void ElectionModule::setLeaderElected() {
    std::lock_guard<std::mutex> lock(this->leader_mutex_);
    this->leader_elected_ = true;
}

void ElectionModule::unsetLeaderElected() {
    std::lock_guard<std::mutex> lock(this->leader_mutex_);
    this->leader_elected_ = false;
}

void ElectionModule::setRandomBallotWaittime() {
    this->new_ballot_waittime_ =
        std::chrono::milliseconds {this->random_distribution_(this->random_engine_)};
    this->sendLogDebug("new_ballot_waittime_ set to {}", this->new_ballot_waittime_.count());
}

void ElectionModule::setRandomElectionTimeout() {
    this->election_timeout_ =
        std::chrono::milliseconds {this->random_distribution_(this->random_engine_)};
    this->sendLogDebug("election_timeout_ set to {}", this->election_timeout_.count());
}

void ElectionModule::setElectionCompleted() {
    this->voting_timer_->cancel();
    std::lock_guard<std::mutex> lock(this->election_completed_mutex_);
    this->election_completed_ = true;
}

void ElectionModule::unsetElectionCompleted() {
    std::lock_guard<std::mutex> lock(this->election_completed_mutex_);
    this->election_completed_ = false;
}

void ElectionModule::setVotingCompleted() {
    std::lock_guard<std::mutex> lock(this->voting_completed_mutex_);
    this->voting_completed_ = true;
}

void ElectionModule::unsetVotingCompleted() {
    std::lock_guard<std::mutex> lock(this->voting_completed_mutex_);
    this->voting_completed_ = false;
}

void ElectionModule::setExternalLeaderElected() {
    std::lock_guard<std::mutex> lock(this->external_leader_mutex_);
    this->external_leader_elected_ = true;
}

void ElectionModule::unsetExternalLeaderElected() {
    std::lock_guard<std::mutex> lock(this->external_leader_mutex_);
    this->external_leader_elected_ = false;
}

void ElectionModule::setLeader(int id) {
    this->setLeaderElected();

    if (id == -1) {
        this->leader_id_ = this->node_->getID();
    } else {
        this->leader_id_ = id;
    }
}

void ElectionModule::setIsTerminated() {
    std::lock_guard<std::mutex> lock(this->terminated_mutex_);
    this->is_terminated_ = true;
}
