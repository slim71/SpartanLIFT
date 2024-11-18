/**
 * @file setters.cpp
 * @author Simone Vollaro (slim71sv@gmail.com)
 * @brief File containing getter methods for the UNSCModule class.
 * @version 1.0.0
 * @date 2024-11-13
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "UNSCModule/unsc.hpp"

/***************************** Getters *****************************/
/**
 * @brief Gets the status of the running_ flag.
 *
 * @return true
 * @return false
 */
bool UNSCModule::getRunningStatus() const {
    std::lock_guard lock(this->running_mutex_);
    return this->running_;
}

/**
 * @brief Retrieves the current offset value used by the module.
 *
 *
 * @return The current offset as an Eigen::Vector3d object.
 */
Eigen::Vector3d UNSCModule::getOffset() const {
    std::lock_guard lock(this->offset_mutex_);
    return this->offset_;
}

/**
 * @brief Retrieves the current position setpoint, if available.
 *
 * @return std::optional<geometry_msgs::msg::Point> The position setpoint, or std::nullopt if not
 * available.
 */
std::optional<geometry_msgs::msg::Point> UNSCModule::getPositionSetpoint() const {
    std::lock_guard lock(this->setpoint_position_mutex_);
    if (this->setpoint_position_ != NAN_point) {
        return this->setpoint_position_;
    }

    return std::nullopt;
}

/**
 * @brief Retrieves the current velocity setpoint, if available.
 *
 * @return std::optional<geometry_msgs::msg::Point> The velocity setpoint, or std::nullopt if not
 * available.
 */
std::optional<geometry_msgs::msg::Point> UNSCModule::getSetpointVelocity() const {
    std::lock_guard lock(this->setpoint_velocity_mutex_);
    if (this->setpoint_velocity_ != NAN_point) {
        return this->setpoint_velocity_;
    }

    return std::nullopt;
}

/**
 * @brief Retrieves the actual target height.
 *
 * @return double The actual target height.
 */
double UNSCModule::getActualTargetHeight() const {
    std::lock_guard lock(this->height_mutex_);
    return this->actual_target_height_;
}

/**
 * @brief Retrieves the current target position, if available.
 *
 * @return std::optional<geometry_msgs::msg::Point> The target position, or std::nullopt if not
 * available.
 */
std::optional<geometry_msgs::msg::Point> UNSCModule::getTargetPosition() const {
    std::lock_guard lock(this->target_position_mutex_);
    if (this->target_position_ != NAN_point) {
        return this->target_position_;
    }

    return std::nullopt;
}

/**
 * @brief Retrieves the IDs of the neighbors.
 *
 * @return std::vector<unsigned int> The list of neighbor IDs.
 */
std::vector<unsigned int> UNSCModule::getNeighborsIDs() const {
    std::lock_guard lock(this->neighbors_mutex_);
    return this->neighbors_;
}

/**
 * @brief Retrieves the desired position of a specific neighbor.
 *
 * @param agent The ID of the neighbor whose desired position is to be retrieved.
 *
 * @return geometry_msgs::msg::Point The desired position of the specified neighbor.
 */
geometry_msgs::msg::Point UNSCModule::getNeighborDesPos(unsigned int agent) {
    try {
        std::lock_guard lock(this->neighbors_despos_mutex_);
        return this->neigh_des_positions_.at(agent);

    } catch (const std::out_of_range&) {
        // If not already present, collect it and THEN return it
        this->collectNeighDesPositions(agent);
        std::lock_guard lock(this->neighbors_despos_mutex_);
        return this->neigh_des_positions_.at(agent);
    }
}

/**
 * @brief Retrieves the ID of the closest agent.
 *
 * @return unsigned int The ID of the closest agent.
 */
unsigned int UNSCModule::getClosestAgent() const {
    std::lock_guard lock(this->closest_agent_mutex_);
    return this->closest_agent_;
}

/**
 * @brief Retrieves the angle to the closest agent.
 *
 * @return double The angle to the closest agent in radians.
 */
double UNSCModule::getClosestAngle() const {
    std::lock_guard lock(this->closest_angle_mutex_);
    return this->closest_angle_;
}

/**
 * @brief Retrieves the fleet order of a specific agent.
 *
 * @param id The ID of the agent whose fleet order is to be retrieved.
 *
 * @return unsigned int The fleet order of the specified agent.
 */
unsigned int UNSCModule::getFleetOrder(unsigned int id) const {
    std::lock_guard lock(this->order_mutex_);
    return this->fleet_order_.at(id);
}

/**
 * @brief Retrieves the amount of time the UAV has been close enough to the target.
 *
 * @return uint64_t The target count.
 */
uint64_t UNSCModule::getTargetCount() const {
    std::lock_guard lock(this->target_count_mutex_);
    return this->near_target_counter_;
}

/**
 * @brief Retrieves the count of stuck incidents encountered.
 *
 * @return uint64_t The stuck count.
 */
uint64_t UNSCModule::getStuckCount() const {
    std::lock_guard lock(this->stuck_count_mutex_);
    return this->stuck_counter_;
}

/**
 * @brief Retrieves the last completed operation ID.
 *
 * @return uint32_t The last completed operation ID.
 */
uint32_t UNSCModule::getLastCompletedOperation() const {
    std::lock_guard lock(this->last_op_mutex_);
    return this->last_op_completed_;
}

/**
 * @brief Retrieves the last distance from the target.
 *
 * @return geometry_msgs::msg::Point The last distance from the target.
 */
geometry_msgs::msg::Point UNSCModule::getLastDistanceFromTarget() const {
    std::lock_guard lock(this->last_dist_mutex_);
    return this->last_dist_;
}

/**
 * @brief Checks the status of the sitl_ready_ flag.
 *
 * @return true
 * @return false
 */
bool UNSCModule::getSimulationReady() {
    std::lock_guard lock(this->sitl_ready_mutex_);
    return this->sitl_ready_;
}
