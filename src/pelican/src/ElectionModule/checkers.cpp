#include "ElectionModule/election.hpp"
#include "PelicanModule/pelican.hpp"

bool ElectionModule::isElectionCompleted() const {
    std::lock_guard lock(this->election_completed_mutex_);
    return this->election_completed_;
}

bool ElectionModule::isVotingCompleted() const {
    std::lock_guard lock(this->voting_completed_mutex_);
    return this->voting_completed_;
}

bool ElectionModule::isLeaderElected() const {
    std::lock_guard lock(this->leader_elected_mutex_);
    return this->leader_elected_;
}
