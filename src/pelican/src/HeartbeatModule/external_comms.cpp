#include "HeartbeatModule/heartbeat.hpp"
#include "PelicanModule/pelican.hpp"

// Considering that external functionalities are not active
// if the main module is not present, everything can throw an error if
// node_ is not set

/*************************** Get methods ***************************/
int HeartbeatModule::gatherAgentID() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->sendLogDebug("Gathering agent ID");
    return this->node_->getID();
}

possible_roles HeartbeatModule::gatherAgentRole() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->sendLogDebug("Gathering agent role");
    return this->node_->getRole();
}

int HeartbeatModule::gatherCurrentTerm() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->sendLogDebug("Gathering current term");
    return this->node_->getCurrentTerm();
}

rclcpp::Time HeartbeatModule::gatherTime() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->sendLogDebug("Gathering time");
    return this->node_->getTime();
}

rclcpp::CallbackGroup::SharedPtr HeartbeatModule::gatherReentrantGroup() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->sendLogDebug("Gathering reentrant group");
    return this->node_->getReentrantGroup();
}

rclcpp::SubscriptionOptions HeartbeatModule::gatherReentrantOptions() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->sendLogDebug("Gathering reentrant options");
    return this->node_->getReentrantOptions();
}

/************* To make other modules carry on an action ************/
void HeartbeatModule::signalTransitionToFollower() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->sendLogDebug("Signaling transition to follower");
    this->node_->commenceFollowerOperations();
}

void HeartbeatModule::signalSetElectionStatus(int64_t id) {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->sendLogDebug("Signaling election status set");
    this->node_->commenceSetElectionStatus(id);
}

void HeartbeatModule::signalResetElectionTimer() {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->sendLogDebug("Signaling election timer reset");
    this->node_->commenceResetElectionTimer();
}
