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
#include "TacMapModule/tacmap.hpp"

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
unsigned int TacMapModule::gatherAgentID() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getID();
}

/**
 * @brief Retrieves the model of the agent.
 *
 * This function accesses and returns the agent model from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return std::string Agent model.
 */
std::string TacMapModule::gatherAgentModel() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getModel();
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
possible_roles TacMapModule::gatherAgentRole() const {
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
int TacMapModule::gatherCurrentTerm() const {
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
rclcpp::Time TacMapModule::gatherTime() const {
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
rclcpp::CallbackGroup::SharedPtr TacMapModule::gatherReentrantGroup() const {
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
rclcpp::SubscriptionOptions TacMapModule::gatherReentrantOptions() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getReentrantOptions();
}

/************* To make other modules carry on an action ************/
/**
 * @brief Signals a new odometry reference for the height compensation.
 *
 * This method checks if the required external module (e.g., the node) exists. If present,
 * it triggers the operation to increase the current term. If the module is missing, an exception is
 * thrown.
 *
 * @throws MissingExternModule If the required external module or node is missing.
 */
void TacMapModule::signalHeightCompensation(double odom_height) const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceHeightCompensation(odom_height);
}

/**
 * @brief Share a new position in the global frame.
 *
 * This method checks if the required external module (e.g., the node) exists. If present,
 * it triggers the operation to increase the current term. If the module is missing, an exception is
 * thrown.
 *
 * @throws MissingExternModule If the required external module or node is missing.
 */
void TacMapModule::signalSharePosition(geometry_msgs::msg::Point pos) {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceSharePosition(pos);
}

/**
 * @brief Signals a transition to the failure mode.
 *
 * This method checks if the required external module (e.g., the node) exists. If present,
 * it triggers the operation to increase the current term. If the module is missing, an exception is
 * thrown.
 *
 * @throws MissingExternModule If the required external module or node is missing.
 */
void TacMapModule::signalTransitionToFailureMode() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->transitionToFailureMode();
}
