#include "PelicanModule/pelican.hpp"
#include "TacMapModule/tacmap.hpp"

/*************************** Get methods ***************************/
unsigned int TacMapModule::gatherAgentID() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getID();
}

std::string TacMapModule::gatherAgentModel() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getModel();
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

/*************************** Set methods ***************************/
// For possible future use
// void TacMapModule::signalSetPoseInfo(float x, float y, float z, float yaw) {
//     this->node_->commenceSetPoseInfo(x, y, z, yaw);
// }
