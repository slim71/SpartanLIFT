#include "PelicanModule/pelican.hpp"
#include "UNSCModule/unsc.hpp"

/*************************** Get methods ***************************/
rclcpp::Time UNSCModule::gatherTime() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getTime();
}

rclcpp::CallbackGroup::SharedPtr UNSCModule::gatherReentrantGroup() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getReentrantGroup();
}

std::optional<px4_msgs::msg::VehicleGlobalPosition> UNSCModule::gatherGlobalPosition() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->requestGlobalPosition();
}

std::optional<px4_msgs::msg::VehicleOdometry> UNSCModule::gatherOdometry() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->requestOdometry();
}

std::optional<px4_msgs::msg::VehicleStatus> UNSCModule::gatherStatus() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

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

    this->node_->commencePublishVehicleCommand(
        command, param1, param2, param3, param4, param5, param6, param7
    );
}

void UNSCModule::signalPublishOffboardControlMode() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commencePublishOffboardControlMode();
}

void UNSCModule::signalPublishTrajectorySetpoint(float x, float y, float z, float yaw) const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commencePublishTrajectorySetpoint(x, y, z, yaw);
}

bool UNSCModule::signalWaitForAck(uint16_t command) const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->commenceWaitForAck(command);
}
