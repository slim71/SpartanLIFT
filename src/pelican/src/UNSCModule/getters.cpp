#include "UNSCModule/unsc.hpp"

/***************************** Getters *****************************/
bool UNSCModule::getRunningStatus() const {
    std::lock_guard lock(this->running_mutex_);
    return this->running_;
}

Eigen::Vector3d UNSCModule::getOffset() const {
    std::lock_guard lock(this->offset_mutex_);
    return this->offset_;
}

std::optional<geometry_msgs::msg::Point> UNSCModule::getPositionSetpoint() const {
    std::lock_guard lock(this->setpoint_position_mutex_);
    if (this->setpoint_position_ != NAN_point) {
        return this->setpoint_position_;
    }

    return std::nullopt;
}

std::optional<geometry_msgs::msg::Point> UNSCModule::getSetpointVelocity() const {
    std::lock_guard lock(this->setpoint_velocity_mutex_);
    if (this->setpoint_velocity_ != NAN_point) {
        return this->setpoint_velocity_;
    }

    return std::nullopt;
}

double UNSCModule::getActualTargetHeight() const {
    std::lock_guard lock(this->height_mutex_);
    return this->actual_target_height_;
}

std::optional<geometry_msgs::msg::Point> UNSCModule::getTargetPosition() const {
    std::lock_guard lock(this->target_position_mutex_);
    if (this->target_position_ != NAN_point) {
        return this->target_position_;
    }

    return std::nullopt;
}
