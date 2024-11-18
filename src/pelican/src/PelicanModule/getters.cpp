/**
 * @file getters.cpp
 * @author Simone Vollaro (slim71sv@gmail.com)
 * @brief File containing getter methods for the PelicanModule class.
 * @version 1.0.0
 * @date 2024-11-13
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "PelicanModule/pelican.hpp"

/************************* Standard getters ***************************/
/**
 * @brief Get the agent ID.
 *
 * @return unsigned int Agent ID.
 */
unsigned int Pelican::getID() const {
    std::lock_guard lock(this->id_mutex_);
    return this->id_;
}

/**
 * @brief Get the model used for the agent.
 *
 * @return std::string Model used.
 */
std::string Pelican::getModel() const {
    return this->model_;
}

/**
 * @brief Get the agent mass.
 *
 * @return double Agent mass.
 */
double Pelican::getMass() const {
    return this->mass_;
}

/**
 * @brief Get the agent role.
 *
 * @return possible_roles Current role of the agent.
 */
possible_roles Pelican::getRole() const {
    return this->role_;
}

/**
 * @brief Get the current term ID.
 *
 * @return unsigned int Term ID.
 */
unsigned int Pelican::getCurrentTerm() const {
    std::lock_guard lock(this->term_mutex_);
    return this->current_term_;
}

/**
 * @brief Retrieve the instance linked to the Pelican object.
 *
 * @return std::shared_ptr<Pelican> The instance itself.
 */
std::shared_ptr<Pelican> Pelican::getInstance() {
    return instance_.lock();
}

/**
 * @brief Get the standard reentrant options used.
 *
 * @return rclcpp::SubscriptionOptions Standard reentrant options.
 */
rclcpp::SubscriptionOptions Pelican::getReentrantOptions() const {
    return this->reentrant_opt_;
}

/**
 * @brief Get the standard reentrant group used.
 *
 * @return rclcpp::CallbackGroup::SharedPtr Standard reentrant group.
 */
rclcpp::CallbackGroup::SharedPtr Pelican::getReentrantGroup() const {
    return this->reentrant_group_;
}

/**
 * @brief Get the exclusive group related to timing operations.
 *
 * @return rclcpp::CallbackGroup::SharedPtr Exclusive group.
 */
rclcpp::CallbackGroup::SharedPtr Pelican::getTimerExclusiveGroup() const {
    return this->timer_exclusive_group_;
}

/**
 * @brief Get the exclusive group related to offboard operations.
 *
 * @return rclcpp::CallbackGroup::SharedPtr Exclusive group.
 */
rclcpp::CallbackGroup::SharedPtr Pelican::getOffboardExclusiveGroup() const {
    return this->offboard_exclusive_group_;
}

/**
 * @brief Get the exclusive group related to rendezvous operations.
 *
 * @return rclcpp::CallbackGroup::SharedPtr Exclusive group.
 */
rclcpp::CallbackGroup::SharedPtr Pelican::getRendezvousExclusiveGroup() const {
    return this->rendezvous_exclusive_group_;
}

/**
 * @brief Get the exclusive group related to formation operations.
 *
 * @return rclcpp::CallbackGroup::SharedPtr Exclusive group.
 */
rclcpp::CallbackGroup::SharedPtr Pelican::getFormationExclusiveGroup() const {
    return this->formation_exclusive_group_;
}

/**
 * @brief Get the exclusive group related to formation timing operations.
 *
 * @return rclcpp::CallbackGroup::SharedPtr Exclusive group.
 */
rclcpp::CallbackGroup::SharedPtr Pelican::getFormationTimerGroup() const {
    return this->formation_timer_group_;
}

/**
 * @brief Get the exclusive group related to triggering new operations.
 *
 * @return rclcpp::CallbackGroup::SharedPtr Exclusive group.
 */
rclcpp::CallbackGroup::SharedPtr Pelican::getOpCompletedExclusiveGroup() const {
    return this->trigger_exclusive_group_;
}

/**
 * @brief Get the exclusive group related to target tracking operations.
 *
 * @return rclcpp::CallbackGroup::SharedPtr Exclusive group.
 */
rclcpp::CallbackGroup::SharedPtr Pelican::getTargetExclusiveGroup() const {
    return this->target_exclusive_group_;
}

/**
 * @brief Get the exclusive group related to height tracking operations.
 *
 * @return rclcpp::CallbackGroup::SharedPtr Exclusive group.
 */
rclcpp::CallbackGroup::SharedPtr Pelican::getHeightExclusiveGroup() const {
    return this->height_exclusive_group_;
}

/**
 * @brief Get the exclusive group related to checks.
 *
 * @return rclcpp::CallbackGroup::SharedPtr Exclusive group.
 */
rclcpp::CallbackGroup::SharedPtr Pelican::getCheckExclusiveGroup() const {
    return this->check_exclusive_group_;
}

/**
 * @brief Get the exclusive group related to point-to-point operations.
 *
 * @return rclcpp::CallbackGroup::SharedPtr Exclusive group.
 */
rclcpp::CallbackGroup::SharedPtr Pelican::getP2PExclusiveGroup() const {
    return this->p2p_exclusive_group_;
}

/**
 * @brief Get the exclusive group related to election operations.
 *
 * @return rclcpp::CallbackGroup::SharedPtr Exclusive group.
 */
rclcpp::CallbackGroup::SharedPtr Pelican::getBallotExclusiveGroup() const {
    return this->ballot_exclusive_group_;
}

/**
 * @brief Get current time.
 *
 * @return rclcpp::Time Current time in Unix timestamp.
 */
rclcpp::Time Pelican::getTime() const {
    return this->now();
}

/**
 * @brief Get the region of interest radius around the agent.
 *
 * @return double Region of interest radius.
 */
double Pelican::getROI() const {
    return this->roi_;
}

/**
 * @brief Get the size of the network discovered.
 *
 * @return unsigned int Network size.
 */
unsigned int Pelican::getNetworkSize() const {
    std::lock_guard lock(this->discovery_mutex_);
    if (this->discovery_vector_.empty()) {
        return 1000; // Simply return a "big" enough number
    } else {
        int s = this->discovery_vector_.size() + 1;
        return s;
    }
}

/**
 * @brief Get the recorded position of a copter.
 *
 * @param id Agent ID.
 * @return geometry_msgs::msg::Point Recorded position of the agent. NaN if none recorded.
 */
geometry_msgs::msg::Point Pelican::getCopterPosition(unsigned int id) const {
    try {
        return this->copters_positions_.at(id);
    } catch (const std::out_of_range&) {
        // ID not known
        return NAN_point;
    }
}

/**
 * @brief Get all agents positions.
 *
 * @return std::vector<unsigned int> Array containing all agents positions.
 */
std::vector<unsigned int> Pelican::getCoptersIDs() const {
    std::vector<unsigned int> ids;
    std::lock_guard lock(this->discovery_mutex_);
    for (auto& elem : this->discovery_vector_) {
        this->sendLogDebug("Accruing ID {}", elem);
        ids.push_back(elem);
    }
    ids.push_back(this->getID()); // Ensure my ID is considered

    return ids;
}

/**
 * @brief Get the desired position to move to.
 *
 * @return geometry_msgs::msg::Point Desired position.
 */
geometry_msgs::msg::Point Pelican::getDesiredPosition() const {
    std::lock_guard lock(this->formation_mutex_);
    return this->des_formation_pos_;
}

/**
 * @brief Get the neighbor desired position.
 *
 * @return geometry_msgs::msg::Point Neighbor desired position.
 */
geometry_msgs::msg::Point Pelican::getNeighborDesiredPosition() const {
    std::lock_guard lock(this->formation_mutex_);
    return this->neigh_des_pos_;
}

/**
 * @brief Get the payload dropoff position.
 *
 * @return geometry_msgs::msg::Point Dropoff position.
 */
geometry_msgs::msg::Point Pelican::getDropoffPosition() const {
    std::lock_guard lock(this->dropoff_mutex_);
    return this->dropoff_position_;
}

/*************************** Status flags *****************************/
/**
 * @brief Check if the agent is the leader of the fleet.
 *
 * @return true
 * @return false
 */
bool Pelican::isLeader() const {
    return (this->getRole() == leader);
}

/**
 * @brief Check if the agent is a follower.
 *
 * @return true
 * @return false
 */
bool Pelican::isFollower() const {
    return (this->getRole() == follower);
}

/**
 * @brief Check if the agent is a candidate.
 *
 * @return true
 * @return false
 */
bool Pelican::isCandidate() const {
    return (this->getRole() == candidate);
}

/**
 * @brief Check the ready_ status of the flag.
 *
 * @return true
 * @return false
 */
bool Pelican::isReady() const {
    return this->ready_;
}

/**
 * @brief Check the flying_ status of the flag.
 *
 * @return true
 * @return false
 */
bool Pelican::isFlying() const {
    std::lock_guard lock(this->flying_mutex_);
    return this->flying_;
}

/**
 * @brief Check the carrying_ status of the flag.
 *
 * @return true
 * @return false
 */
bool Pelican::isCarrying() const {
    std::lock_guard lock(this->carrying_mutex_);
    return this->carrying_;
}

/**
 * @brief Check if the last command has been executed by the fleet.
 *
 * @return true
 * @return false
 */
bool Pelican::isLastCmdExecuted() const {
    std::lock_guard lock(this->last_cmd_result_mutex_);
    return this->last_cmd_result_;
}

/**
 * @brief Check if the formation has achieved the desired configuration.
 *
 * @return true
 * @return false
 */
bool Pelican::isFormationAchieved() const {
    std::lock_guard lock(this->form_achieved_mutex_);
    return this->formation_achieved_;
}
