#include "PelicanModule/pelican.hpp"
#include "TacMapModule/tacmap.hpp"

// Considering that external functionalities are not active
// if the main module is not present, everything can throw an error if
// node_ is not set

/*************************** Get methods ***************************/
int TacMapModule::gatherAgentID() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getID();
}

possible_roles TacMapModule::gatherAgentRole() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getRole();
}

int TacMapModule::gatherCurrentTerm() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getCurrentTerm();
}

rclcpp::Time TacMapModule::gatherTime() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getTime();
}

rclcpp::CallbackGroup::SharedPtr TacMapModule::gatherReentrantGroup() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getReentrantGroup();
}

rclcpp::SubscriptionOptions TacMapModule::gatherReentrantOptions() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getReentrantOptions();
}
