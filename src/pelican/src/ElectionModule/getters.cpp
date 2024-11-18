/**
 * @file getters.cpp
 * @author Simone Vollaro (slim71sv@gmail.com)
 * @brief File containing getter methods for the ElectionModule class.
 * @version 1.0.0
 * @date 2024-11-13
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "ElectionModule/election.hpp"

/**
 * @brief Get the leader ID.
 *
 * @return unsigned int Recorded leader ID.
 */
unsigned int ElectionModule::getLeaderID() const {
    std::lock_guard lock(this->leader_id_mutex_);
    return this->leader_id_;
}
