/**
 * @file setters.cpp
 * @author Simone Vollaro (slim71sv@gmail.com)
 * @brief File containing setter methods for the UNSCModule class.
 * @version 1.0.0
 * @date 2024-11-13
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "UNSCModule/unsc.hpp"

/***************************** Setters *****************************/
/**
 * @brief Set 2D position setpoint to follow.
 *
 * @param p Setpoint to follow.
 */
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

/**
 * @brief Set height setpoint to follow.
 *
 * @param h Height reference.
 */
void UNSCModule::setHeightSetpoint(double h) {
    std::lock_guard lock(this->setpoint_position_mutex_);
    this->sendLogDebug("Setting height setpoint to {}", h);
    this->setpoint_position_.z = h;
}

/**
 * @brief Set velocity setpoint to follow.
 *
 * @param v Velocity setpoint.
 * @param max_vel Maximum threshold to cap the velocity reference.
 */
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

/**
 * @brief Store reference of the actual height.
 *
 * @param height Current actual height.
 */
void UNSCModule::setActualTargetHeight(double height) {
    std::lock_guard lock(this->height_mutex_);
    this->sendLogDebug("Setting target height to {}", height);
    this->actual_target_height_ = height;
}

/**
 * @brief Set target position to move to.
 *
 * @param p Target position.
 */
void UNSCModule::setTargetPosition(geometry_msgs::msg::Point p) {
    std::lock_guard lock(this->target_position_mutex_);
    this->sendLogDebug("Setting target position to {}", p);
    this->target_position_ = p;
}

/**
 * @brief Set neighbor_gathered_ flag to False.
 *
 */
void UNSCModule::unsetNeighborGathered() {
    std::lock_guard lock(this->formation_cv_mutex_);
    this->neighbor_gathered_ = false;
}

/**
 * @brief Store the closest agent's ID.
 *
 * @param id Closest agent's ID.
 */
void UNSCModule::setClosestAgent(unsigned int id) {
    std::lock_guard lock(this->closest_agent_mutex_);
    this->closest_agent_ = id;
}

/**
 * @brief Store info about the angle between this agent and the closest one.
 *
 * @param angle Angle between us and the closest agent.
 */
void UNSCModule::setClosestAngle(double angle) {
    std::lock_guard lock(this->closest_angle_mutex_);
    this->closest_angle_ = angle;
}

/**
 * @brief Store info about the order of an agent in the formation.
 *
 * @param id Agent ID.
 * @param order Order in the fleet formation.
 */
void UNSCModule::setFleetOrder(unsigned int id, unsigned int order) {
    std::lock_guard lock(this->order_mutex_);
    this->fleet_order_[id] = order;
}

/**
 * @brief Increase the count of terms spent near the target position.
 *
 */
void UNSCModule::increaseTargetCount() {
    std::lock_guard lock(this->target_count_mutex_);
    this->near_target_counter_++;
}

/**
 * @brief Reset the count of terms spent near the target position.
 *
 */
void UNSCModule::resetTargetCount() {
    std::lock_guard lock(this->target_count_mutex_);
    this->near_target_counter_ = 0;
}

/**
 * @brief Increase the count of terms spent in a stuck condition.
 *
 */
void UNSCModule::increaseStuckCount() {
    std::lock_guard lock(this->stuck_count_mutex_);
    this->stuck_counter_++;
}

/**
 * @brief Reset the count of terms spent in a stuck condition.
 *
 */
void UNSCModule::resetStuckCount() {
    std::lock_guard lock(this->stuck_count_mutex_);
    this->stuck_counter_ = 0;
}

/**
 * @brief Store the last recorded distance from the target
 *
 * @param d Distance to store.
 */
void UNSCModule::setLastDistanceFromTarget(geometry_msgs::msg::Point d) {
    std::lock_guard lock(this->last_dist_mutex_);
    this->last_dist_ = d;
}
