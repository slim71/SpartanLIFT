/**
 * @file checkers.cpp
 * @author Simone Vollaro (slim71sv@gmail.com)
 * @brief Methods used to check flag in the module.
 * @version 1.0.0
 * @date 2024-11-14
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "ElectionModule/election.hpp"
#include "PelicanModule/pelican.hpp"

/**
 * @brief Check and return the status of the election_completed_ flag.
 *
 * @return true
 * @return false
 */
bool ElectionModule::isElectionCompleted() const {
    std::lock_guard lock(this->election_completed_mutex_);
    return this->election_completed_;
}

/**
 * @brief Check and return the status of the voting_completed_ flag.
 *
 * @return true
 * @return false
 */
bool ElectionModule::isVotingCompleted() const {
    std::lock_guard lock(this->voting_completed_mutex_);
    return this->voting_completed_;
}

/**
 * @brief Check and return the status of the leader_elected_ flag.
 *
 * @return true
 * @return false
 */
bool ElectionModule::isLeaderElected() const {
    std::lock_guard lock(this->leader_elected_mutex_);
    return this->leader_elected_;
}
