#include "pelican.hpp"
#include "types.hpp"

void Pelican::setLeaderElected() {
    std::lock_guard<std::mutex> lock(this->leader_mutex_);
    this->leader_elected_ = true;
}

void Pelican::unsetLeaderElected() {
    std::lock_guard<std::mutex> lock(this->leader_mutex_);
    this->leader_elected_ = false;
}

void Pelican::setRandomBallotWaittime() {
    this->new_ballot_waittime_ =
        std::chrono::milliseconds {this->random_distribution_(this->random_engine_)};
    this->sendLogDebug("new_ballot_waittime_ set to {}", this->new_ballot_waittime_.count());
}

void Pelican::setRandomElectionTimeout() {
    this->election_timeout_ =
        std::chrono::milliseconds {this->random_distribution_(this->random_engine_)};
    this->sendLogDebug("election_timeout_ set to {}", this->election_timeout_.count());
}

void Pelican::setRole(possible_roles r) {
    this->sendLogDebug("Setting role to {}", static_cast<int>(r));
    this->role_ = r;
}

void Pelican::setElectionCompleted() {
    this->voting_timer_->cancel();
    std::lock_guard<std::mutex> lock(this->election_completed_mutex_);
    this->election_completed_ = true;
}

void Pelican::unsetElectionCompleted() {
    std::lock_guard<std::mutex> lock(this->election_completed_mutex_);
    this->election_completed_ = false;
}

void Pelican::setVotingCompleted() {
    std::lock_guard<std::mutex> lock(this->voting_completed_mutex_);
    this->voting_completed_ = true;
}

void Pelican::unsetVotingCompleted() {
    std::lock_guard<std::mutex> lock(this->voting_completed_mutex_);
    this->voting_completed_ = false;
}

void Pelican::setExternalLeaderElected() {
    std::lock_guard<std::mutex> lock(this->external_leader_mutex_);
    this->external_leader_elected_ = true;
}

void Pelican::unsetExternalLeaderElected() {
    std::lock_guard<std::mutex> lock(this->external_leader_mutex_);
    this->external_leader_elected_ = false;
}

void Pelican::increaseCurrentTerm() {
    this->current_term_++;
}

void Pelican::setMass(double m) {
    this->mass_ = m;
}

void Pelican::setInstance(rclcpp::Node::SharedPtr instance) {
    instance_ = std::static_pointer_cast<Pelican>(instance);
}

void Pelican::setLeader(int id) {
    this->setLeaderElected();

    if (id == -1) {
        this->leader_id_ = this->getID();
    } else {
        this->leader_id_ = id;
    }
}

void Pelican::setIsTerminated() {
    std::lock_guard<std::mutex> lock(this->terminated_mutex_);
    this->is_terminated_ = true;
}
