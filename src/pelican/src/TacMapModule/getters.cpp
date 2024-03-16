#include "TacMapModule/tacmap.hpp"

/*************************** Get methods ***************************/
std::optional<px4_msgs::msg::VehicleGlobalPosition> TacMapModule::getGlobalPosition() {
    this->globalpos_mutex_.lock();
    int g_length = this->globalpos_buffer_.size();
    this->globalpos_mutex_.unlock();

    if (g_length <= 0)
        return std::nullopt;

    std::lock_guard<std::mutex> lock(this->globalpos_mutex_);
    return globalpos_buffer_.back();
}

std::optional<px4_msgs::msg::VehicleOdometry> TacMapModule::getOdometry() {
    this->ned_odometry_mutex_.lock();
    int o_length = this->ned_odometry_buffer_.size();
    this->ned_odometry_mutex_.unlock();

    if (o_length <= 0)
        return std::nullopt;

    std::lock_guard<std::mutex> lock(this->ned_odometry_mutex_);
    return ned_odometry_buffer_.back();
}

std::optional<px4_msgs::msg::VehicleStatus> TacMapModule::getStatus() {
    this->status_mutex_.lock();
    int s_length = this->status_buffer_.size();
    this->status_mutex_.unlock();

    if (s_length <= 0)
        return std::nullopt;

    std::lock_guard<std::mutex> lock(this->status_mutex_);
    return this->status_buffer_.back();
}

std::optional<px4_msgs::msg::VehicleCommandAck> TacMapModule::getCommanderAck() {
    std::lock_guard<std::mutex> lock(this->commander_ack_mutex_);
    return this->last_commander_ack_;
}

bool TacMapModule::getRunningStatus() const {
    std::lock_guard<std::mutex> lock(this->running_mutex_);
    return this->running_;
}
