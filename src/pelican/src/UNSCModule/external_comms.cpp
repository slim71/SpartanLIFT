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

rclcpp::CallbackGroup::SharedPtr UNSCModule::gatherOffboardExclusiveGroup() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getOffboardExclusiveGroup();
}

rclcpp::CallbackGroup::SharedPtr UNSCModule::gatherRendezvousExclusiveGroup() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getRendezvousExclusiveGroup();
}

std::optional<px4_msgs::msg::VehicleOdometry> UNSCModule::gatherOdometry() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->requestNEDOdometry();
}

std::optional<px4_msgs::msg::VehicleStatus> UNSCModule::gatherStatus() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->requestStatus();
}

std::optional<geometry_msgs::msg::Point> UNSCModule::gatherTargetPosition() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getSetpointPosition();
}

std::optional<geometry_msgs::msg::Point> UNSCModule::gatherSetpointVelocity() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getSetpointVelocity();
}

std::optional<geometry_msgs::msg::Point> UNSCModule::gatherDesiredPosition() const {
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

geometry_msgs::msg::Point UNSCModule::gatherCopterPosition(unsigned int id) const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getCopterPosition(id);
}

double UNSCModule::gatherROI() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getROI();
}

double UNSCModule::gatherCollisionRadius() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getCollisionRadius();
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
    geometry_msgs::msg::Point pos, geometry_msgs::msg::Point vel
) const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commencePublishTrajectorySetpoint(pos, vel);
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

    return this->node_->commenceCheckOffboardEngagement();
}

void UNSCModule::signalSetSetpointPosition(geometry_msgs::msg::Point p) const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->commenceSetSetpointPosition(p);
}

void UNSCModule::signalSetSetpointVelocity(geometry_msgs::msg::Point v) const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->commenceSetSetpointVelocity(v);
}
