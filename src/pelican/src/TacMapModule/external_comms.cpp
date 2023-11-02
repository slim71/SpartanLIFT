#include "PelicanModule/pelican.hpp"
#include "TacMapModule/tacmap.hpp"
#include "types.hpp"

// Considering that external functionalities are not active
// if the main module is not present, everything can throw an error if
// node_ is not set

/*************************** Get methods ***************************/
int TacMapModule::gatherAgentID() const {
    if (!this->node_) {
        throw EXTERNAL_OFF;
    }

    return this->node_->getID();
}

possible_roles TacMapModule::gatherAgentRole() const {
    if (!this->node_) {
        throw EXTERNAL_OFF;
    }

    return this->node_->getRole();
}

int TacMapModule::gatherCurrentTerm() const {
    if (!this->node_) {
        throw EXTERNAL_OFF;
    }

    return this->node_->getCurrentTerm();
}

rclcpp::Time TacMapModule::gatherTime() const {
    if (!this->node_) {
        throw EXTERNAL_OFF;
    }

    return this->node_->now();
}

rclcpp::CallbackGroup::SharedPtr TacMapModule::gatherReentrantGroup() const {
    if (!this->node_) {
        throw EXTERNAL_OFF;
    }

    return this->node_->getReentrantGroup();
}

rclcpp::SubscriptionOptions TacMapModule::gatherReentrantOptions() const {
    if (!this->node_) {
        throw EXTERNAL_OFF;
    }

    return this->node_->getReentrantOptions();
}

std::optional<px4_msgs::msg::VehicleGlobalPosition> TacMapModule::getGlobalPosition() {
    this->globalpos_mutex_.lock();
    int g_length = this->globalpos_buffer_.size();
    this->globalpos_mutex_.unlock();

    this->sendLogDebug("g_length={}", g_length);
    if (g_length <= 0)
        return std::nullopt;

    std::lock_guard<std::mutex> lock(this->globalpos_mutex_);
    return globalpos_buffer_.back();
}

std::optional<px4_msgs::msg::VehicleOdometry> TacMapModule::getOdometry() {
    this->odometry_mutex_.lock();
    int g_length = this->odometry_buffer_.size();
    this->odometry_mutex_.unlock();

    if (g_length <= 0)
        return std::nullopt;

    std::lock_guard<std::mutex> lock(this->odometry_mutex_);
    return odometry_buffer_.back();
}

std::optional<px4_msgs::msg::VehicleCommandAck> TacMapModule::getAck() {
    std::lock_guard<std::mutex> lock(this->ack_mutex_);
    return this->last_ack_;
}
