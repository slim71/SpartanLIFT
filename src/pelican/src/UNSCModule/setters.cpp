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

void UNSCModule::setVelocitySetpoint(geometry_msgs::msg::Point v, double max_vel) {
    // Velocity is capped to keep copter's movement smoother and
    // to allow a better control of setpoint tracking
    auto v_capped = v;
    if (abs(v.x) > max_vel)
        v_capped.set__x(signum(v.x) * max_vel);
    if (abs(v.y) > max_vel)
        v_capped.set__y(signum(v.y) * max_vel);
    v_capped.set__z(std::nanf(""));

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

void UNSCModule::setClosestAgent(unsigned int id) {
    std::lock_guard lock(this->closest_agent_mutex_);
    this->closest_agent_ = id;
}

void UNSCModule::setClosestAngle(double angle) {
    std::lock_guard lock(this->closest_angle_mutex_);
    this->closest_angle_ = angle;
}

void UNSCModule::setFleetOrder(unsigned int id, unsigned int order) {
    std::lock_guard lock(this->order_mutex_);
    this->fleet_order_[id] = order;
}

void UNSCModule::increaseTargetCount() {
    std::lock_guard lock(this->target_count_mutex_);
    this->near_target_counter_++;
}

void UNSCModule::resetTargetCount() {
    std::lock_guard lock(this->target_count_mutex_);
    this->near_target_counter_ = 0;
}

void UNSCModule::increaseStuckCount() {
    std::lock_guard lock(this->stuck_count_mutex_);
    this->stuck_counter_++;
}

void UNSCModule::resetStuckCount() {
    std::lock_guard lock(this->stuck_count_mutex_);
    this->stuck_counter_ = 0;
}

void UNSCModule::setLastDistanceFromTarget(geometry_msgs::msg::Point d) {
    std::lock_guard lock(this->last_dist_mutex_);
    this->last_dist_ = d;
}
