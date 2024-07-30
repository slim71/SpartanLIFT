#include "PelicanModule/pelican.hpp"
#include "UNSCModule/unsc.hpp"

/************************** Gather methods *************************/
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

rclcpp::CallbackGroup::SharedPtr UNSCModule::gatherFormationExclusiveGroup() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getFormationExclusiveGroup();
}

std::optional<nav_msgs::msg::Odometry> UNSCModule::gatherENUOdometry() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->requestENUOdometry();
}

std::optional<px4_msgs::msg::VehicleStatus> UNSCModule::gatherStatus() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->requestStatus();
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

geometry_msgs::msg::Point UNSCModule::gatherDesiredPosition() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getDesiredPosition();
}

unsigned int UNSCModule::gatherLeaderID() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->requestLeaderID();
}

std::vector<unsigned int> UNSCModule::gatherCoptersIDs() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getCoptersIDs();
}

geometry_msgs::msg::Point UNSCModule::gatherNeighborDesiredPosition() {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->getNeighborDesiredPosition();
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

void UNSCModule::signalSetReferenceHeight(double height) {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceSetActualTargetHeight(height);
}

void UNSCModule::signalCargoAttachment() {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->cargoAttachment();
}

void UNSCModule::signalSendDesiredFormationPositions(
    std::unordered_map<unsigned int, geometry_msgs::msg::Point> pos
) {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->sendDesiredFormationPositions(pos);
}

void UNSCModule::signalAskDesPosToNeighbor(unsigned int id) {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->askDesPosToNeighbor(id);
}

void UNSCModule::signalNotifyAgentInFormation() {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->node_->commenceNotifyAgentInFormation();
}

/*************************** Flag checks ***************************/
bool UNSCModule::confirmAgentIsLeader() const {
    if (!this->node_) {
        throw MissingExternModule();
    }

    return this->node_->isLeader();
}
