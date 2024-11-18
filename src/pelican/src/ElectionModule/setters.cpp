/**
 * @file setters.cpp
 * @author Simone Vollaro (slim71sv@gmail.com)
 * @brief File containing setter methods for the ElectionModule class.
 * @version 1.0.0
 * @date 2024-11-13
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "ElectionModule/election.hpp"

/************************** Public methods *************************/
/**
 * @brief Resets the election timer to its initial state.
 *
 */
void ElectionModule::resetElectionTimer() {
    resetTimer(this->election_timer_);
}

/**
 * @brief Resets the subscription pointer to the leader election topic.
 *
 */
void ElectionModule::resetSubscriptions() {
    resetSharedPointer(this->sub_to_leader_election_topic_);
}

/**
 * @brief Sets the status of the election by marking it as completed and recording the leader.
 *
 * @param id The ID of the elected leader.
 */
void ElectionModule::setElectionStatus(int id) {
    this->setLeaderElected();
    this->setVotingCompleted();
    this->setElectionCompleted();
    this->setLeader(id);
}

/************************* Private methods *************************/

/**
 * @brief Marks the election as completed.
 *
 */
void ElectionModule::setElectionCompleted() {
    std::lock_guard lock(this->election_completed_mutex_);
    this->election_completed_ = true;
}

/**
 * @brief Resets the election completed status.
 *
 */
void ElectionModule::unsetElectionCompleted() {
    std::lock_guard lock(this->election_completed_mutex_);
    this->election_completed_ = false;
}

/**
 * @brief Marks the voting process as completed.
 *
 */
void ElectionModule::setVotingCompleted() {
    std::lock_guard lock(this->voting_completed_mutex_);
    this->voting_completed_ = true;
}

/**
 * @brief Resets the voting completed status.
 *
 */
void ElectionModule::unsetVotingCompleted() {
    std::lock_guard lock(this->voting_completed_mutex_);
    this->voting_completed_ = false;
}

/**
 * @brief Sets a random election timeout value.
 *
 */
void ElectionModule::setRandomElectionTimeout() {
    this->election_timeout_ =
        std::chrono::milliseconds {this->random_distribution_(this->random_engine_)};
    this->sendLogDebug("election_timeout_ set to {} ms", this->election_timeout_.count());
}

/**
 * @brief Clears the election status, resetting all relevant flags.
 *
 */
void ElectionModule::clearElectionStatus() {
    this->unsetLeaderElected();
    this->unsetVotingCompleted();
    this->unsetElectionCompleted();
    this->setLeader();
}

/**
 * @brief Sets the elected leader's ID.
 *
 * @param id The ID of the elected leader; a value of -1 indicates no leader.
 */
void ElectionModule::setLeader(int id) {
    std::lock_guard lock(this->leader_id_mutex_);
    // id=0 means election is not finished or has been reset
    if (id >= 0) {
        this->leader_id_ = id;
    } else {
        this->sendLogWarning("Tried to setup a negative ID! Setting it at -1 as indication...");
        this->leader_id_ = -1;
    }
}

/**
 * @brief Marks the leader as elected.
 *
 */
void ElectionModule::setLeaderElected() {
    std::lock_guard lock(this->leader_elected_mutex_);
    this->leader_elected_ = true;
}

/**
 * @brief Resets the leader elected status.
 *
 */
void ElectionModule::unsetLeaderElected() {
    std::lock_guard lock(this->leader_elected_mutex_);
    this->leader_elected_ = false;
}
