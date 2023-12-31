#include "ElectionModule/election.hpp"
#include "PelicanModule/pelican.hpp"

// Considering that external functionalities are not active
// if the main module is not present, everything can throw an error if
// node_ is not set

/*************************** Get methods ***************************/
int ElectionModule::gatherAgentID() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getID();
}

double ElectionModule::gatherAgentMass() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getMass();
}

possible_roles ElectionModule::gatherAgentRole() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getRole();
}

int ElectionModule::gatherCurrentTerm() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getCurrentTerm();
}

int ElectionModule::gatherNumberOfHbs() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->requestNumberOfHbs();
}

heartbeat ElectionModule::gatherLastHb() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->requestLastHb();
}

rclcpp::CallbackGroup::SharedPtr ElectionModule::gatherReentrantGroup() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getReentrantGroup();
}

rclcpp::SubscriptionOptions ElectionModule::gatherReentrantOptions() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getReentrantOptions();
}

/************************** Check methods **************************/
bool ElectionModule::confirmAgentIsCandidate() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->isCandidate();
}

/************* To make other modules carry on an action ************/
void ElectionModule::signalNewTerm() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceIncreaseCurrentTerm();
}

void ElectionModule::signalTransitionToLeader() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceLeaderOperations();
}

void ElectionModule::signalTransitionToCandidate() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceCandidateOperations();
}

void ElectionModule::signalTransitionToFollower() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceFollowerOperations();
}
