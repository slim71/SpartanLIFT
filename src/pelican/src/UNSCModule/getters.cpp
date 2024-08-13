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

std::vector<unsigned int> UNSCModule::getNeighborsIDs() const {
    std::lock_guard lock(this->neighbors_mutex_);
    return this->neighbors_;
}

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

unsigned int UNSCModule::getClosestAgent() const {
    std::lock_guard lock(this->closest_agent_mutex_);
    return this->closest_agent_;
}

double UNSCModule::getClosestAngle() const {
    std::lock_guard lock(this->closest_angle_mutex_);
    return this->closest_angle_;
}

unsigned int UNSCModule::getFleetOrder(unsigned int id) const {
    std::lock_guard lock(this->order_mutex_);
    return this->fleet_order_.at(id);
}

uint64_t UNSCModule::getTargetCount() const {
    std::lock_guard lock(this->target_count_mutex_);
    return this->near_target_counter_;
}
