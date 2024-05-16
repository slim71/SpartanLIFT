#include "UNSCModule/unsc.hpp"

/***************************** Setters *****************************/
void UNSCModule::setSetpointPosition(geometry_msgs::msg::Point p) {
    std::lock_guard lock(this->setpoint_position_mutex_);
    this->setpoint_position_ = p;
}

void UNSCModule::setSetpointVelocity(geometry_msgs::msg::Point v) {
    // Velocity is capped to keep copter's movement smoother and
    // to allow a better control of setpoint tracking
    std::lock_guard lock(this->setpoint_velocity_mutex_);
    auto v_capped = v;
    if (v.x > 1)
        v_capped.set__x(1);
    if (v.y > 1)
        v_capped.set__y(1);
    if (v.z > 1)
        v_capped.set__z(1);
    this->setpoint_velocity_ = v_capped;
}

void UNSCModule::setActualTargetHeight(double height) {
    std::lock_guard lock(this->height_mutex_);
    this->actual_target_height_ = height;
}

void UNSCModule::setTargetPosition(geometry_msgs::msg::Point p) {
    std::lock_guard lock(this->target_position_mutex_);
    this->target_position_ = p;
}
