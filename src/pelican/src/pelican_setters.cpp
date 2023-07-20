#include "pelican.hpp"

void PelicanUnit::setLeaderElected() {
    std::lock_guard<std::mutex> lock(this->leader_mutex_);
    this->leader_elected_ = true;
}

void PelicanUnit::unsetLeaderElected() {
    std::lock_guard<std::mutex> lock(this->leader_mutex_);
    this->leader_elected_ = false;
}

void PelicanUnit::setRandomBallotWaittime() {
    // DELETE: mutex? not needed
    this->new_ballot_waittime_ = std::chrono::milliseconds { this->random_distribution_(this->random_engine_) };
    this->logDebug("Set new_ballot_waittime_ to {}", this->new_ballot_waittime_.count());
}

void PelicanUnit::setRandomElectionTimeout() {
    // DELETE: mutex? not needed
    this->election_timeout_ = std::chrono::milliseconds { this->random_distribution_(this->random_engine_) };
    this->logDebug("Set election_timeout_ to {}", this->election_timeout_.count());
}

void PelicanUnit::setRole(possible_roles r) {
    this->logDebug("Setting role to {}", static_cast<int>(r));
    this->role_ = r;
}

void PelicanUnit::setElectionTimedOut() {
    this->logInfo("Election timed out!");
    this->election_timer_->cancel(); // Cancel the wall_timer used to call this function
    std::lock_guard<std::mutex> lock(this->election_timedout_mutex_);
    this->election_timed_out_ = true;
}

void PelicanUnit::unsetElectionTimedOut() {
    std::lock_guard<std::mutex> lock(this->election_timedout_mutex_);
    this->election_timed_out_ = false;
}

void PelicanUnit::setElectionCompleted() {
    this->voting_timer->cancel();
    std::lock_guard<std::mutex> lock(this->election_completed_mutex_);
    this->election_completed_ = true;
}

void PelicanUnit::unsetElectionCompleted() {
    std::lock_guard<std::mutex> lock(this->election_completed_mutex_);
    this->election_completed_ = false;
}

void PelicanUnit::setVotingCompleted() {
    std::lock_guard<std::mutex> lock(this->voting_completed_mutex_);
    this->voting_completed_ = true;
}

void PelicanUnit::unsetVotingCompleted() {
    std::lock_guard<std::mutex> lock(this->voting_completed_mutex_);
    this->voting_completed_ = false;
}

void PelicanUnit::setExternalLeaderElected() {
    std::lock_guard<std::mutex> lock(this->external_leader_mutex_);
    this->external_leader_elected_ = true;
}

void PelicanUnit::unsetExternalLeaderElected() {
    std::lock_guard<std::mutex> lock(this->external_leader_mutex_);
    this->external_leader_elected_ = false;
}

void PelicanUnit::increaseCurrentTerm() {
    this->current_term_++;
}

void PelicanUnit::setMass(double m) {
    this->mass_ = m;
}

void PelicanUnit::setInstance(rclcpp::Node::SharedPtr instance) {
    instance_ = std::static_pointer_cast<PelicanUnit>(instance);
}

void PelicanUnit::setLeader(int id = -1) {
    this->setLeaderElected();

    // TODO: sync with other nodes first?
    if (id == -1) {
        this->leader_id_ = this->getID();
    } else {
        this->leader_id_ = id;
    }
}