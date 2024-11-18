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
#include "HeartbeatModule/heartbeat.hpp"
#include "PelicanModule/pelican.hpp"

/*************************** Get methods ***************************/
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
unsigned int HeartbeatModule::gatherAgentID() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getID();
}

/**
 * @brief Retrieves the agent role.
 *
 * This function accesses and returns the agent role from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return possible_roles Current role of the agent.
 */
possible_roles HeartbeatModule::gatherAgentRole() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getRole();
}

/**
 * @brief Retrieves the current term ID.
 *
 * This function accesses and returns the current term ID from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return unsigned int Current term.
 */
unsigned int HeartbeatModule::gatherCurrentTerm() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getCurrentTerm();
}

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
rclcpp::Time HeartbeatModule::gatherTime() const {
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
rclcpp::CallbackGroup::SharedPtr HeartbeatModule::gatherReentrantGroup() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getReentrantGroup();
}

/**
 * @brief Retrieves the standard reentrant options.
 *
 * This function accesses and returns the standard reentrant options from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return rclcpp::SubscriptionOptions Standard reentrant options.
 */
rclcpp::SubscriptionOptions HeartbeatModule::gatherReentrantOptions() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getReentrantOptions();
}

/************* To make other modules carry on an action ************/
/**
 * @brief Signals a transition to follower status.
 *
 * This method checks if the required external module (e.g., the node) exists. If present,
 * it triggers the operation to increase the current term. If the module is missing, an exception is
 * thrown.
 *
 * @throws MissingExternModule If the required external module or node is missing.
 */
void HeartbeatModule::signalTransitionToFollower() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceFollowerOperations();
}

/**
 * @brief Signals a new election status, with a new leader.
 *
 * @param id Leader ID.
 *
 * This method checks if the required external module (e.g., the node) exists. If present,
 * it triggers the operation to increase the current term. If the module is missing, an exception is
 * thrown.
 *
 * @throws MissingExternModule If the required external module or node is missing.
 */
void HeartbeatModule::signalSetElectionStatus(int64_t id) {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceSetElectionStatus(id);
}

/**
 * @brief Signals a reset of the election timer.
 *
 * This method checks if the required external module (e.g., the node) exists. If present,
 * it triggers the operation to increase the current term. If the module is missing, an exception is
 * thrown.
 *
 * @throws MissingExternModule If the required external module or node is missing.
 */
void HeartbeatModule::signalResetElectionTimer() {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceResetElectionTimer();
}

/**
 * @brief Signals the system to set a new term ID.
 *
 * @param term Term ID to set.
 *
 * This method checks if the required external module (e.g., the node) exists. If present,
 * it triggers the operation to increase the current term. If the module is missing, an exception is
 * thrown.
 *
 * @throws MissingExternModule If the required external module or node is missing.
 */
void HeartbeatModule::signalSetTerm(uint64_t term) {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceSetTerm(term);
}
