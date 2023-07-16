#include "pelican.hpp"

bool PelicanUnit::checkElectionTimedOut() {
    std::lock_guard<std::mutex> lock(this->election_timedout_mutex_);
    return this->election_timed_out_;
}

bool PelicanUnit::checkElectionCompleted() {
    // Ensure safe access to election_timed_out
    std::lock_guard<std::mutex> lock(this->election_completed_mutex_);
    return this->election_completed_;
}

bool PelicanUnit::checkVotingCompleted() {
    std::lock_guard<std::mutex> lock(this->voting_completed_mutex_);
    return this->voting_completed_;
}

bool PelicanUnit::checkExternalLeaderElected() {
    std::lock_guard<std::mutex> lock(this->external_leader_mutex_);
    return this->external_leader_elected_;
}

bool PelicanUnit::checkLeaderElected() {
    std::lock_guard<std::mutex> lock(this->leader_mutex_);
    return this->leader_elected_;
}
