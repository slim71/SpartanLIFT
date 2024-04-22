#include "HeartbeatModule/heartbeat.hpp"
#include "PelicanModule/pelican.hpp"

/*************************** Get methods ***************************/
unsigned int HeartbeatModule::gatherAgentID() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getID();
}

possible_roles HeartbeatModule::gatherAgentRole() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getRole();
}

unsigned int HeartbeatModule::gatherCurrentTerm() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getCurrentTerm();
}

rclcpp::Time HeartbeatModule::gatherTime() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getTime();
}

rclcpp::CallbackGroup::SharedPtr HeartbeatModule::gatherReentrantGroup() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getReentrantGroup();
}

rclcpp::SubscriptionOptions HeartbeatModule::gatherReentrantOptions() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getReentrantOptions();
}

/************* To make other modules carry on an action ************/
void HeartbeatModule::signalTransitionToFollower() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceFollowerOperations();
}

void HeartbeatModule::signalSetElectionStatus(int64_t id) {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceSetElectionStatus(id);
}

void HeartbeatModule::signalResetElectionTimer() {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceResetElectionTimer();
}

void HeartbeatModule::signalSetTerm(uint64_t term) {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceSetTerm(term);
}
