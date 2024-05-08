#include "TacMapModule/tacmap.hpp"

/*************************** Get methods ***************************/
std::optional<nav_msgs::msg::Odometry> TacMapModule::getENUOdometry() const {
    std::lock_guard<std::mutex> lock(this->enu_odometry_mutex_);

    if (this->enu_odometry_buffer_.empty())
        return std::nullopt;

    return enu_odometry_buffer_.back();
}

std::optional<px4_msgs::msg::VehicleStatus> TacMapModule::getStatus() const {
    std::lock_guard<std::mutex> lock(this->status_mutex_);

    if (this->status_buffer_.empty())
        return std::nullopt;

    return this->status_buffer_.back();
}

std::optional<px4_msgs::msg::VehicleCommandAck> TacMapModule::getCommanderAck() const {
    std::lock_guard<std::mutex> lock(this->commander_ack_mutex_);
    return this->last_commander_ack_;
}

bool TacMapModule::getRunningStatus() const {
    std::lock_guard<std::mutex> lock(this->running_mutex_);
    return this->running_;
}
