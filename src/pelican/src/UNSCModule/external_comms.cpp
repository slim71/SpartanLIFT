/**
 * @file external_comms.cpp
 * @author Simone Vollaro (slim71sv@gmail.com)
 * @brief Methods concerned with intra-modules communications.
 * @version 1.0.0
 * @date 2024-11-17
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "PelicanModule/pelican.hpp"
#include "UNSCModule/unsc.hpp"

/************************** Gather methods *************************/
/**
 * @brief Retrieves the current time in unix timestamp.
 *
 * This function accesses and returns the current time from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return rclcpp::Time Current time.
 */
rclcpp::Time UNSCModule::gatherTime() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getTime();
}

/**
 * @brief Retrieves the standard reentrant group.
 *
 * This function accesses and returns the standard reentrant group from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return rclcpp::CallbackGroup::SharedPtr Standard reentrant group.
 */
rclcpp::CallbackGroup::SharedPtr UNSCModule::gatherReentrantGroup() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getReentrantGroup();
}

/**
 * @brief Retrieves the exclusive group related to offboard operations.
 *
 * This function accesses and returns the offboard exclusive group from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return rclcpp::CallbackGroup::SharedPtr Offboard exclusive group.
 */
rclcpp::CallbackGroup::SharedPtr UNSCModule::gatherOffboardExclusiveGroup() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getOffboardExclusiveGroup();
}

/**
 * @brief Retrieves the exclusive group related to rendezvous operations.
 *
 * This function accesses and returns the rendezvous exclusive group from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return rclcpp::CallbackGroup::SharedPtr Rendezvous exclusive group.
 */
rclcpp::CallbackGroup::SharedPtr UNSCModule::gatherRendezvousExclusiveGroup() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getRendezvousExclusiveGroup();
}

/**
 * @brief Retrieves the exclusive group related to formation operations.
 *
 * This function accesses and returns the formation exclusive group from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return rclcpp::CallbackGroup::SharedPtr Formation exclusive group.
 */
rclcpp::CallbackGroup::SharedPtr UNSCModule::gatherFormationExclusiveGroup() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getFormationExclusiveGroup();
}

/**
 * @brief Retrieves the exclusive group related to operation completion.
 *
 * This function accesses and returns the operation completion exclusive group from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return rclcpp::CallbackGroup::SharedPtr Operation completion exclusive group.
 */
rclcpp::CallbackGroup::SharedPtr UNSCModule::gatherOpCompletedExclusiveGroup() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getOpCompletedExclusiveGroup();
}

/**
 * @brief Retrieves the exclusive group related to target tracking operations.
 *
 * This function accesses and returns the target tracking exclusive group from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return rclcpp::CallbackGroup::SharedPtr Target tracking exclusive group.
 */
rclcpp::CallbackGroup::SharedPtr UNSCModule::gatherTargetExclusiveGroup() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getTargetExclusiveGroup();
}

/**
 * @brief Retrieves the exclusive group related to height tracking operations.
 *
 * This function accesses and returns the height tracking exclusive group from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return rclcpp::CallbackGroup::SharedPtr Height tracking exclusive group.
 */
rclcpp::CallbackGroup::SharedPtr UNSCModule::gatherHeightExclusiveGroup() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getHeightExclusiveGroup();
}

/**
 * @brief Retrieves the exclusive group related to formation achieved checks.
 *
 * This function accesses and returns the formation achieved checks exclusive group from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return rclcpp::CallbackGroup::SharedPtr Formation achieved checks exclusive group.
 */
rclcpp::CallbackGroup::SharedPtr UNSCModule::gatherCheckExclusiveGroup() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getCheckExclusiveGroup();
}

/**
 * @brief Retrieves the exclusive group related to point to point operations.
 *
 * This function accesses and returns the point to point exclusive group from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return rclcpp::CallbackGroup::SharedPtr Point to point exclusive group.
 */
rclcpp::CallbackGroup::SharedPtr UNSCModule::gatherP2PExclusiveGroup() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getP2PExclusiveGroup();
}

/**
 * @brief Retrieves last odometry data in ENU frame.
 *
 * This function accesses and returns ENU odometry data from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return std::optional<nav_msgs::msg::Odometry> ENU odometry data.
 */
std::optional<nav_msgs::msg::Odometry> UNSCModule::gatherENUOdometry() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->requestENUOdometry();
}

/**
 * @brief Retrieves status data.
 *
 * This function accesses and returns status data from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return std::optional<px4_msgs::msg::VehicleStatus> Status data.
 */
std::optional<px4_msgs::msg::VehicleStatus> UNSCModule::gatherStatus() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->requestStatus();
}

/**
 * @brief Retrieves the size of the recognized network.
 *
 * This function accesses and returns the network size from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return unsigned int Network size.
 */
unsigned int UNSCModule::gatherNetworkSize() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getNetworkSize();
}

/**
 * @brief Retrieves the agent ID.
 *
 * This function accesses and returns the agent ID from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return unsigned int ID of the agent.
 */
unsigned int UNSCModule::gatherAgentID() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getID();
}

/**
 * @brief Retrieves the last stored position related to the specified agent.
 *
 * This function accesses and returns the copter position from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return geometry_msgs::msg::Point Last stored copter positon.
 */
geometry_msgs::msg::Point UNSCModule::gatherCopterPosition(unsigned int id) const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getCopterPosition(id);
}

/**
 * @brief Retrieves the region of interest configured radius.
 *
 * This function accesses and returns the radius of the region of interest from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return double Radius of the region of interest.
 */
double UNSCModule::gatherROI() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getROI();
}

/**
 * @brief Retrieves the agent's desired position.
 *
 * This function accesses and returns the desired position from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return geometry_msgs::msg::Point Agent's desired position.
 */
geometry_msgs::msg::Point UNSCModule::gatherDesiredPosition() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getDesiredPosition();
}

/**
 * @brief Retrieves the leader ID.
 *
 * This function accesses and returns the leader ID from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return unsigned int Leader ID.
 */
unsigned int UNSCModule::gatherLeaderID() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->requestLeaderID();
}

/**
 * @brief Retrieves the IDs of all copters in the network.
 *
 * This function accesses and returns the copters' IDs from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return std::vector<unsigned int> Copters' IDs.
 */
std::vector<unsigned int> UNSCModule::gatherCoptersIDs() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getCoptersIDs();
}

/**
 * @brief Retrieves the neighbor desired position.
 *
 * This function accesses and returns the neighbor desired position from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return geometry_msgs::msg::Point Neighbor desired position.
 */
geometry_msgs::msg::Point UNSCModule::gatherNeighborDesiredPosition() {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getNeighborDesiredPosition();
}

/**
 * @brief Retrieves the payload dropoff position.
 *
 * This function accesses and returns the payload dropoff position from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return geometry_msgs::msg::Point Payload dropoff position.
 */
geometry_msgs::msg::Point UNSCModule::gatherDropoffPosition() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getDropoffPosition();
}

/************* To make other modules carry on an action ************/
/**
 * @brief Passes a new command for the PX4 commander to the appropriate module, along with its
 * parameters.
 *
 * @param command
 * @param param1
 * @param param2
 * @param param3
 * @param param4
 * @param param5
 * @param param6
 * @param param7
 *
 * This method checks if the required external module (e.g., the node) exists. If present,
 * it triggers the operation to increase the current term. If the module is missing, an exception is
 * thrown.
 *
 * @throws MissingExternModule If the required external module or node is missing.
 */
void UNSCModule::signalPublishVehicleCommand(
    uint16_t command, float param1, float param2, float param3, float param4, float param5,
    float param6, float param7
) const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commencePublishVehicleCommand(
        command, param1, param2, param3, param4, param5, param6, param7
    );
}

/**
 * @brief Signals a new message to publish for the PX4 offboard mode setting.
 *
 * This method checks if the required external module (e.g., the node) exists. If present,
 * it triggers the operation to increase the current term. If the module is missing, an exception is
 * thrown.
 *
 * @throws MissingExternModule If the required external module or node is missing.
 */
void UNSCModule::signalPublishOffboardControlMode() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commencePublishOffboardControlMode();
}

/**
 * @brief Signals a new trajectory setpoint.
 *
 * @param pos Position setpoint.
 * @param vel Velocity setpoint.
 *
 * This method checks if the required external module (e.g., the node) exists. If present,
 * it triggers the operation to increase the current term. If the module is missing, an exception is
 * thrown.
 *
 * @throws MissingExternModule If the required external module or node is missing.
 */
void UNSCModule::signalPublishTrajectorySetpoint(
    geometry_msgs::msg::Point pos, geometry_msgs::msg::Point vel
) const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commencePublishTrajectorySetpoint(pos, vel);
}

/**
 * @brief Triggers a new wait cycle until the ack to the specified command is received.
 *
 * @param command Command for which to received an ack.
 *
 * This method checks if the required external module (e.g., the node) exists. If present,
 * it triggers the operation to increase the current term. If the module is missing, an exception is
 * thrown.
 *
 * @throws MissingExternModule If the required external module or node is missing.
 */
bool UNSCModule::signalWaitForCommanderAck(uint16_t command) const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->commenceWaitForCommanderAck(command);
}

/**
 * @brief Triggers a new check for the engagement of the offboard mode.
 *
 * This method checks if the required external module (e.g., the node) exists. If present,
 * it triggers the operation to increase the current term. If the module is missing, an exception is
 * thrown.
 *
 * @throws MissingExternModule If the required external module or node is missing.
 */
bool UNSCModule::signalCheckOffboardEngagement() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->commenceCheckOffboardEngagement();
}

/**
 * @brief Signals a new target height.
 *
 * @param height Target height.
 *
 * This method checks if the required external module (e.g., the node) exists. If present,
 * it triggers the operation to increase the current term. If the module is missing, an exception is
 * thrown.
 *
 * @throws MissingExternModule If the required external module or node is missing.
 */
void UNSCModule::signalSetActualTargetHeight(double height) {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceSetActualTargetHeight(height);
}

/**
 * @brief Signals a new cargo attachment/detachment.
 *
 * @param attach Whether to attach or detach the cargo.
 *
 * This method checks if the required external module (e.g., the node) exists. If present,
 * it triggers the operation to increase the current term. If the module is missing, an exception is
 * thrown.
 *
 * @throws MissingExternModule If the required external module or node is missing.
 */
void UNSCModule::signalCargoAttachment(bool attach) {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceCargoAttachment(attach);
}

/**
 * @brief Signals new formation position to be distributed in the network.
 *
 * @param pos Map of agent ID-position to be shared.
 *
 * This method checks if the required external module (e.g., the node) exists. If present,
 * it triggers the operation to increase the current term. If the module is missing, an exception is
 * thrown.
 *
 * @throws MissingExternModule If the required external module or node is missing.
 */
void UNSCModule::signalSendDesiredFormationPositions(
    std::unordered_map<unsigned int, geometry_msgs::msg::Point> pos
) {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceSendDesiredFormationPositions(pos);
}

/**
 * @brief Forward the request of neighbor position.
 *
 * This method checks if the required external module (e.g., the node) exists. If present,
 * it triggers the operation to increase the current term. If the module is missing, an exception is
 * thrown.
 *
 * @throws MissingExternModule If the required external module or node is missing.
 */
void UNSCModule::signalAskDesPosToNeighbor(unsigned int id) {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceAskDesPosToNeighbor(id);
}

/**
 * @brief Triggers notification of agent in formation.
 *
 * This method checks if the required external module (e.g., the node) exists. If present,
 * it triggers the operation to increase the current term. If the module is missing, an exception is
 * thrown.
 *
 * @throws MissingExternModule If the required external module or node is missing.
 */
void UNSCModule::signalNotifyAgentInFormation() {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceNotifyAgentInFormation();
}

/**
 * @brief Send synchronization signal about a completed operation.
 *
 * This method checks if the required external module (e.g., the node) exists. If present,
 * it triggers the operation to increase the current term. If the module is missing, an exception is
 * thrown.
 *
 * @throws MissingExternModule If the required external module or node is missing.
 */
void UNSCModule::signalSyncCompletedOp(uint32_t cmd) {
    this->sendLogDebug("signaling trigger {}", cmd);
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceSyncCompletedOp(cmd);
}

/**
 * @brief Signals the reset of the carrying status.
 *
 * This method checks if the required external module (e.g., the node) exists. If present,
 * it triggers the operation to increase the current term. If the module is missing, an exception is
 * thrown.
 *
 * @throws MissingExternModule If the required external module or node is missing.
 */
void UNSCModule::signalUnsetCarryingStatus() {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceUnsetCarryingStatus();
}

/**
 * @brief Signals the reset of the formation status.
 *
 * This method checks if the required external module (e.g., the node) exists. If present,
 * it triggers the operation to increase the current term. If the module is missing, an exception is
 * thrown.
 *
 * @throws MissingExternModule If the required external module or node is missing.
 */
void UNSCModule::signalUnsetFormationAchieved() {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceUnsetFormationAchieved();
}

/**
 * @brief Triggers the clearing of the array containing agents in formation.
 *
 * This method checks if the required external module (e.g., the node) exists. If present,
 * it triggers the operation to increase the current term. If the module is missing, an exception is
 * thrown.
 *
 * @throws MissingExternModule If the required external module or node is missing.
 */
void UNSCModule::signalEmptyFormationResults() {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceEmptyFormationResults();
}

/*************************** Flag checks ***************************/
/**
 * @brief Check if the agent is currently the leader in the network.
 *
 * @return true
 * @return false
 */
bool UNSCModule::confirmAgentIsLeader() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->isLeader();
}

/**
 * @brief Check if the formation configuration has been achieved.
 *
 * @return true
 * @return false
 */
bool UNSCModule::confirmFormationAchieved() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->isFormationAchieved();
}
