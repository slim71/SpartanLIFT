#include "PelicanModule/pelican.hpp"
#include "UNSCModule/unsc.hpp"

// Considering that external functionalities are not active
// if the main module is not present, everything can throw an error if
// node_ is not set

/*************************** Get methods ***************************/
rclcpp::Time UNSCModule::gatherTime() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->sendLogDebug("Gathering time");
    return this->node_->getTime();
}

rclcpp::CallbackGroup::SharedPtr UNSCModule::gatherReentrantGroup() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->sendLogDebug("Gathering reentrant group");
    return this->node_->getReentrantGroup();
}

std::optional<px4_msgs::msg::VehicleGlobalPosition> UNSCModule::gatherGlobalPosition() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->sendLogDebug("Gathering global position");
    return this->node_->requestGlobalPosition();
}

std::optional<px4_msgs::msg::VehicleOdometry> UNSCModule::gatherOdometry() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->sendLogDebug("Gathering odometry");
    return this->node_->requestOdometry();
}

std::optional<px4_msgs::msg::VehicleCommandAck> UNSCModule::gatherAck() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->sendLogDebug("Gathering ack");
    return this->node_->requestAck();
}

std::optional<px4_msgs::msg::VehicleStatus> UNSCModule::gatherStatus() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->sendLogDebug("Gathering status");
    return this->node_->requestStatus();
}

/************* To make other modules carry on an action ************/
void UNSCModule::signalPublishVehicleCommand(
    uint16_t command, float param1, float param2, float param3, float param4, float param5,
    float param6, float param7
) const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->sendLogDebug("Signaling vehicle command publication");
    this->node_->commencePublishVehicleCommand(
        command, param1, param2, param3, param4, param5, param6, param7
    );
}
