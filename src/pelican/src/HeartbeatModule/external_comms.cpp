#include "HeartbeatModule/heartbeat.hpp"
#include "PelicanModule/pelican.hpp"
#include "types.hpp"

// Considering that external functionalities are not active
// if the main module is not present, everything can throw an error if
// node_ is not set

/*************************** Get methods ***************************/
int HeartbeatModule::gatherAgentID() const {
    if (!this->node_) {
        throw EXTERNAL_OFF;
    }

    return this->node_->getID();
}

possible_roles HeartbeatModule::gatherAgentRole() const {
    if (!this->node_) {
        throw EXTERNAL_OFF;
    }

    return this->node_->getRole();
}

int HeartbeatModule::gatherCurrentTerm() const {
    if (!this->node_) {
        throw EXTERNAL_OFF;
    }

    return this->node_->getCurrentTerm();
}

rclcpp::Time HeartbeatModule::gatherTime() const {
    if (!this->node_) {
        throw EXTERNAL_OFF;
    }

    return this->node_->now();
}

rclcpp::CallbackGroup::SharedPtr HeartbeatModule::gatherReentrantGroup() const {
    if (!this->node_) {
        throw EXTERNAL_OFF;
    }

    return this->node_->getReentrantGroup();
}

rclcpp::SubscriptionOptions HeartbeatModule::gatherReentrantOptions() const {
    if (!this->node_) {
        throw EXTERNAL_OFF;
    }

    return this->node_->getReentrantOptions();
}

/************* To make other modules carry on an action ************/
void HeartbeatModule::signalTransitionToFollower() const {
    if (!this->node_) {
        throw EXTERNAL_OFF;
    }

    this->node_->commenceFollowerOperations();
}

void HeartbeatModule::signalSetElectionStatus(int64_t id) {
    if (!this->node_) {
        throw EXTERNAL_OFF;
    }

    this->node_->commenceSetElectionStatus(id);
}

void HeartbeatModule::signalResetElectionTimer() {
    if (!this->node_) {
        throw EXTERNAL_OFF;
    }

    this->node_->commenceResetElectionTimer();
}
