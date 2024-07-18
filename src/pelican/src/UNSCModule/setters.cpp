#include "UNSCModule/unsc.hpp"

/***************************** Setters *****************************/
void UNSCModule::setPositionSetpoint(geometry_msgs::msg::Point p) {
    std::lock_guard lock(this->setpoint_position_mutex_);
    this->sendLogDebug("Setting position setpoint to {}", p);
    this->setpoint_position_.x = p.x;
    this->setpoint_position_.y = p.y;
    // The z component is handled only at first, to be sure the
    // setpoint followed is always valid
    if (std::isnan(this->setpoint_position_.z))
        this->setpoint_position_.z = p.z;
}

void UNSCModule::setHeightSetpoint(double h) {
    std::lock_guard lock(this->setpoint_position_mutex_);
    this->sendLogDebug("Setting height setpoint to {}", h);
    this->setpoint_position_.z = h;
}

void UNSCModule::setVelocitySetpoint(geometry_msgs::msg::Point v) {
    // Velocity is capped to keep copter's movement smoother and
    // to allow a better control of setpoint tracking
    auto v_capped = v;
    if (v.x > 1)
        v_capped.set__x(1);
    if (v.y > 1)
        v_capped.set__y(1);
    if (v.z > 1)
        v_capped.set__z(1);

    std::lock_guard lock(this->setpoint_velocity_mutex_);
    this->sendLogDebug("Setting velocity setpoint to {}", v_capped);
    this->setpoint_velocity_ = v_capped;
}

void UNSCModule::setActualTargetHeight(double height) {
    std::lock_guard lock(this->height_mutex_);
    this->sendLogDebug("Setting target height to {}", height);
    this->actual_target_height_ = height;
}

void UNSCModule::setTargetPosition(geometry_msgs::msg::Point p) {
    std::lock_guard lock(this->target_position_mutex_);
    this->sendLogDebug("Setting target position to {}", p);
    this->target_position_ = p;
}

void UNSCModule::unsetNeighborGathered() {
    std::lock_guard lock(this->formation_cv_mutex_);
    this->neighbor_gathered_ = false;
}

void UNSCModule::setNeighborGathered() {
    std::lock_guard lock(this->formation_cv_mutex_);
    this->neighbor_gathered_ = true;
}
