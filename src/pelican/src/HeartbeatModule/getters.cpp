/**
 * @file getters.cpp
 * @author Simone Vollaro (slim71sv@gmail.com)
 * @brief File containing getter methods for the HeartbeatModule class.
 * @version 1.0.0
 * @date 2024-11-13
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "HeartbeatModule/heartbeat.hpp"

/**
 * @brief Get the number of heartbeats stored.
 *
 * @return int Number of heartbeats stored.
 */
int HeartbeatModule::getNumberOfHbs() const {
    std::lock_guard lock(this->hbs_mutex_);
    return this->received_hbs_.size();
}

/**
 * @brief Get the last heartbeat recorded.
 *
 * @return heartbeat Last heartbeat recorded.
 */
heartbeat HeartbeatModule::getLastHb() const {
    std::lock_guard lock(this->hbs_mutex_);
    if (!this->received_hbs_.empty())
        return this->received_hbs_.back();
    else
        return ERROR_HB;
}

/**
 * @brief Get the maximum number of storable heartbeat.
 *
 * @return int Max number of storable heartbeats.
 */
int HeartbeatModule::getMaxHbs() const {
    std::lock_guard lock(this->hbs_mutex_);
    return this->received_hbs_.size();
}
