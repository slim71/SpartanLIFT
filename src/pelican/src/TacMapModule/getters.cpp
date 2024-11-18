/**
 * @file getters.cpp
 * @author Simone Vollaro (slim71sv@gmail.com)
 * @brief File containing getter methods for the TacMapModule class.
 * @version 1.0.0
 * @date 2024-11-13
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "TacMapModule/tacmap.hpp"

/*************************** Get methods ***************************/
/**
 * @brief Get the last odometry stored in the ENU frame.
 *
 * @return std::optional<nav_msgs::msg::Odometry>
 */
std::optional<nav_msgs::msg::Odometry> TacMapModule::getENUOdometry() const {
    std::lock_guard lock(this->enu_odometry_mutex_);

    if (this->enu_odometry_buffer_.empty())
        return std::nullopt;

    return enu_odometry_buffer_.back();
}

/**
 * @brief Get the last status message stored.
 *
 * @return std::optional<px4_msgs::msg::VehicleStatus>
 */
std::optional<px4_msgs::msg::VehicleStatus> TacMapModule::getStatus() const {
    std::lock_guard lock(this->status_mutex_);

    if (this->status_buffer_.empty())
        return std::nullopt;

    return this->status_buffer_.back();
}

/**
 * @brief Get the last ack stored for the command sent.
 *
 * @return std::optional<px4_msgs::msg::VehicleCommandAck>
 */
std::optional<px4_msgs::msg::VehicleCommandAck> TacMapModule::getCommanderAck() const {
    std::lock_guard lock(this->commander_ack_mutex_);
    return this->last_commander_ack_;
}

/**
 * @brief Get the status of the running_ flag.
 *
 * @return true
 * @return false
 */
bool TacMapModule::getRunningStatus() const {
    std::lock_guard lock(this->running_mutex_);
    return this->running_;
}
