#include "TacMapModule/tacmap.hpp"

// Considering that external functionalities are not active
// if the main module is not present, everything can throw an error if
// node_ is not set

/*************************** Get methods ***************************/
std::optional<px4_msgs::msg::VehicleGlobalPosition> TacMapModule::getGlobalPosition() {
    this->globalpos_mutex_.lock();
    int g_length = this->globalpos_buffer_.size();
    this->globalpos_mutex_.unlock();

    this->sendLogDebug("Getting global position: g_length={}", g_length);
    if (g_length <= 0)
        return std::nullopt;

    std::lock_guard<std::mutex> lock(this->globalpos_mutex_);
    return globalpos_buffer_.back();
}

std::optional<px4_msgs::msg::VehicleOdometry> TacMapModule::getOdometry() {
    this->odometry_mutex_.lock();
    int o_length = this->odometry_buffer_.size();
    this->odometry_mutex_.unlock();

    this->sendLogDebug("Getting odometry: o_length={}", o_length);

    if (o_length <= 0)
        return std::nullopt;

    std::lock_guard<std::mutex> lock(this->odometry_mutex_);
    return odometry_buffer_.back();
}

std::optional<px4_msgs::msg::VehicleStatus> TacMapModule::getStatus() {
    this->status_mutex_.lock();
    int s_length = this->status_buffer_.size();
    this->status_mutex_.unlock();

    this->sendLogDebug("Getting last status data: s_length={}", s_length);

    if (s_length <= 0)
        return std::nullopt;

    std::lock_guard<std::mutex> lock(this->status_mutex_);
    return this->status_buffer_.back();
}

std::optional<px4_msgs::msg::VehicleCommandAck> TacMapModule::getAck() {
    this->sendLogDebug("Getting last received ack");
    std::lock_guard<std::mutex> lock(this->ack_mutex_);
    return this->last_ack_;
}
