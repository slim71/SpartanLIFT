#include "ElectionModule/election.hpp"

/************************** Public methods *************************/
void ElectionModule::resetElectionTimer() {
    resetTimer(this->election_timer_);
}

void ElectionModule::resetSubscriptions() {
    resetSharedPointer(this->sub_to_leader_election_topic_);
}

void ElectionModule::setElectionStatus(int id) {
    this->setExternalLeaderElected();
    this->setLeaderElected();
    this->setVotingCompleted();
    this->setElectionCompleted();
    this->setLeader(id);
}

/************************* Private methods *************************/
void ElectionModule::setElectionCompleted() {
    std::lock_guard lock(this->election_completed_mutex_);
    this->election_completed_ = true;
}

void ElectionModule::unsetElectionCompleted() {
    std::lock_guard lock(this->election_completed_mutex_);
    this->election_completed_ = false;
}

void ElectionModule::setVotingCompleted() {
    std::lock_guard lock(this->voting_completed_mutex_);
    this->voting_completed_ = true;
}

void ElectionModule::unsetVotingCompleted() {
    std::lock_guard lock(this->voting_completed_mutex_);
    this->voting_completed_ = false;
}

void ElectionModule::setRandomElectionTimeout() {
    this->election_timeout_ =
        std::chrono::milliseconds {this->random_distribution_(this->random_engine_)};
    this->sendLogDebug("election_timeout_ set to {} ms", this->election_timeout_.count());
}

void ElectionModule::clearElectionStatus() {
    this->unsetExternalLeaderElected();
    this->unsetLeaderElected();
    this->unsetVotingCompleted();
    this->unsetElectionCompleted();
    this->setLeader();
}

void ElectionModule::setExternalLeaderElected() {
    std::lock_guard lock(this->external_leader_mutex_);
    this->external_leader_elected_ = true;
}

void ElectionModule::unsetExternalLeaderElected() {
    std::lock_guard lock(this->external_leader_mutex_);
    this->external_leader_elected_ = false;
}

void ElectionModule::setLeader(int id) {
    // id=0 means election is not finished or has been reset
    if (id >= 0) {
        this->leader_id_ = id;
    } else {
        this->sendLogWarning("Tried to setup a negative ID! Setting it at -1 as indication...");
        this->leader_id_ = -1;
    }
}

void ElectionModule::setLeaderElected() {
    std::lock_guard lock(this->leader_mutex_);
    this->leader_elected_ = true;
}

void ElectionModule::unsetLeaderElected() {
    std::lock_guard lock(this->leader_mutex_);
    this->leader_elected_ = false;
}
