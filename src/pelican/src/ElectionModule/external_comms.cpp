#include "ElectionModule/election.hpp"
#include "PelicanModule/pelican.hpp"

/*************************** Get methods ***************************/
unsigned int ElectionModule::gatherAgentID() const {
    if (!this->node_) {
        throw MissingExternModule();
    }
    this->sendLogDebug("gathering id");

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

unsigned int ElectionModule::gatherCurrentTerm() const {
    if (!this->node_) {
        throw MissingExternModule();
    }
    this->sendLogDebug("gathering term");

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

unsigned int ElectionModule::gatherNetworkSize() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getNetworkSize();
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
void ElectionModule::signalIncreaseTerm() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceIncreaseCurrentTerm();
}

void ElectionModule::signalSetTerm(uint64_t term) {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceSetTerm(term);
}

void ElectionModule::signalTransitionToLeader() {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceLeaderOperations();
}

void ElectionModule::signalTransitionToCandidate() {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceCandidateOperations();
}

void ElectionModule::signalTransitionToFollower() {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceFollowerOperations();
}
