#include "pelican.hpp"

bool Pelican::checkElectionCompleted() const {
    // Ensure safe access to election_timed_out
    std::lock_guard<std::mutex> lock(this->election_completed_mutex_);
    return this->election_completed_;
}

bool Pelican::checkVotingCompleted() const {
    std::lock_guard<std::mutex> lock(this->voting_completed_mutex_);
    return this->voting_completed_;
}

bool Pelican::checkExternalLeaderElected() const {
    std::lock_guard<std::mutex> lock(this->external_leader_mutex_);
    return this->external_leader_elected_;
}

bool Pelican::checkLeaderElected() const {
    std::lock_guard<std::mutex> lock(this->leader_mutex_);
    return this->leader_elected_;
}

bool Pelican::checkIsTerminated() const {
    std::lock_guard<std::mutex> lock(this->terminated_mutex_);
    return this->is_terminated_;
}
