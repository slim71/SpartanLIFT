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

// TODO: include yaw or change name?
std::optional<std::vector<float>> UNSCModule::gatherTargetPose() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getSetpointPosition();
}

std::optional<std::vector<float>> UNSCModule::gatherTargetVelocity() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getTargetVelocity();
}

std::optional<std::vector<float>> UNSCModule::gatherDesiredPose() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getTargetPosition();
}

unsigned int UNSCModule::gatherNetworkSize() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getNetworkSize();
}

unsigned int UNSCModule::gatherAgentID() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getID();
}

geometry_msgs::msg::Point UNSCModule::gatherCopterPosition(unsigned int id) {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getCopterPosition(id);
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

void UNSCModule::signalPublishTrajectorySetpoint(
    float x, float y, float z, float yaw, float vx, float vy
) const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commencePublishTrajectorySetpoint(x, y, z, yaw, vx, vy);
}

bool UNSCModule::signalWaitForCommanderAck(uint16_t command) const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->commenceWaitForCommanderAck(command);
}

bool UNSCModule::signalCheckOffboardEngagement() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->initiateCheckOffboardEngagement();
}

void UNSCModule::signalSetSetpointPosition(float x, float y, float z) const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->initiateSetSetpointPosition(x, y, z);
}

void UNSCModule::signalSetTargetVelocity(float vx, float vy) const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->initiateSetTargetVelocity(vx, vy);
}
