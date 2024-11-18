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
#include "ElectionModule/election.hpp"
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
unsigned int ElectionModule::gatherAgentID() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getID();
}

/**
 * @brief Retrieves the agent mass.
 *
 * This function accesses and returns the agent mass from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return double Mass of the agent.
 */
double ElectionModule::gatherAgentMass() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getMass();
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
possible_roles ElectionModule::gatherAgentRole() const {
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
unsigned int ElectionModule::gatherCurrentTerm() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getCurrentTerm();
}

/**
 * @brief Retrieves the number of heartbeats received.
 *
 * This function accesses and returns the number of received heartbeats from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return int Number of received heartbeats.
 */
int ElectionModule::gatherNumberOfHbs() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->requestNumberOfHbs();
}

/**
 * @brief Retrieves the last received heartbeat.
 *
 * This function accesses and returns the last received heartbeat from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return heartbeat Last heartbeat received.
 */
heartbeat ElectionModule::gatherLastHb() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->requestLastHb();
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
unsigned int ElectionModule::gatherNetworkSize() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getNetworkSize();
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
rclcpp::CallbackGroup::SharedPtr ElectionModule::gatherReentrantGroup() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getReentrantGroup();
}

/**
 * @brief Retrieves the exclusive group related to the election system.
 *
 * This function accesses and returns the ballot exclusive group from the
 * `node_` external module. If `node_` is not available, it throws an exception
 * to signal the missing external dependency.
 *
 * @throws MissingExternModule If the `node_` module is not set or accessible.
 *
 * @return rclcpp::CallbackGroup::SharedPtr Ballot exclusive group.
 */
rclcpp::CallbackGroup::SharedPtr ElectionModule::gatherBallotExclusiveGroup() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getBallotExclusiveGroup();
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
rclcpp::SubscriptionOptions ElectionModule::gatherReentrantOptions() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getReentrantOptions();
}

/************************** Check methods **************************/
/**
 * @brief Check if the agent is currently a candidate.
 *
 * @return true
 * @return false
 */
bool ElectionModule::confirmAgentIsCandidate() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->isCandidate();
}

/************* To make other modules carry on an action ************/
/**
 * @brief Signals the system to increase the term ID.
 *
 * This method checks if the required external module (e.g., the node) exists. If present,
 * it triggers the operation to increase the current term. If the module is missing, an exception is
 * thrown.
 *
 * @throws MissingExternModule If the required external module or node is missing.
 */
void ElectionModule::signalIncreaseTerm() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceIncreaseCurrentTerm();
}

/**
 * @brief Signals the system to set a specific term ID.
 *
 * @param term Term ID to set.
 *
 * This method checks if the required external module (e.g., the node) exists. If present,
 * it triggers the operation to increase the current term. If the module is missing, an exception is
 * thrown.
 *
 * @throws MissingExternModule If the required external module or node is missing.
 */
void ElectionModule::signalSetTerm(uint64_t term) {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceSetTerm(term);
}

/**
 * @brief Signals a transition to leader state.
 *
 * This method checks if the required external module (e.g., the node) exists. If present,
 * it triggers the operation to increase the current term. If the module is missing, an exception is
 * thrown.
 *
 * @throws MissingExternModule If the required external module or node is missing.
 */
void ElectionModule::signalTransitionToLeader() {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceLeaderOperations();
}

/**
 * @brief Signals a transition to candidate state.
 *
 * This method checks if the required external module (e.g., the node) exists. If present,
 * it triggers the operation to increase the current term. If the module is missing, an exception is
 * thrown.
 *
 * @throws MissingExternModule If the required external module or node is missing.
 */
void ElectionModule::signalTransitionToCandidate() {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceCandidateOperations();
}

/**
 * @brief Signals a transition to follower state.
 *
 * This method checks if the required external module (e.g., the node) exists. If present,
 * it triggers the operation to increase the current term. If the module is missing, an exception is
 * thrown.
 *
 * @throws MissingExternModule If the required external module or node is missing.
 */
void ElectionModule::signalTransitionToFollower() {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceFollowerOperations();
}

/**
 * @brief Pass the ID of a recognized agent to the overall system.
 *
 * @param id ID of the agent discovered in the network.
 *
 * This method checks if the required external module (e.g., the node) exists. If present,
 * it triggers the operation to increase the current term. If the module is missing, an exception is
 * thrown.
 *
 * @throws MissingExternModule If the required external module or node is missing.
 */
void ElectionModule::signalStoreAttendance(unsigned int id) {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceStoreAttendance(id);
}
