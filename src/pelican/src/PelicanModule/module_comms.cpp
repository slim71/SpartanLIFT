#include "PelicanModule/pelican.hpp"

/************************* Heartbeat Module ************************/
heartbeat Pelican::requestLastHb() {
    return this->hb_core_.getLastHb();
}

int Pelican::requestNumberOfHbs() {
    return this->hb_core_.getNumberOfHbs();
}

void Pelican::commenceStopHeartbeatService() {
    this->hb_core_.stopService();
}

/************************* Election Module *************************/
void Pelican::commenceSetElectionStatus(int i) {
    this->el_core_.setElectionStatus(i);
}

void Pelican::commenceResetElectionTimer() {
    this->el_core_.resetElectionTimer();
}

void Pelican::commenceStopElectionService() {
    this->el_core_.stopService();
}

unsigned int Pelican::initiateGetLeaderID() {
    return this->el_core_.getLeaderID();
}

/**************************** UNSC Module *****************************/
bool Pelican::initiateArm() {
    return this->unsc_core_.arm();
}

bool Pelican::initiateSetHome() {
    return this->unsc_core_.setHome();
}

bool Pelican::initiateTakeoff() {
    return this->unsc_core_.takeoff();
}

bool Pelican::initiateLand() {
    return this->unsc_core_.land();
}

bool Pelican::initiateReturnToLaunchPosition() {
    return this->unsc_core_.returnToLaunchPosition();
}

void Pelican::initiateOffboardMode() {
    this->unsc_core_.activateOffboardMode();
}

void Pelican::commenceStopUNSCService() {
    this->unsc_core_.stopService();
}

/*************************** TacMap Module ****************************/
void Pelican::commencePublishVehicleCommand(
    uint16_t command, float param1, float param2, float param3, float param4, float param5,
    float param6, float param7
) {
    this->tac_core_.publishVehicleCommand(
        command, param1, param2, param3, param4, param5, param6, param7
    );
}

void Pelican::commencePublishOffboardControlMode() {
    this->tac_core_.publishOffboardControlMode();
}

void Pelican::commencePublishTrajectorySetpoint(
    geometry_msgs::msg::Point pos, geometry_msgs::msg::Point vel
) {
    this->tac_core_.publishTrajectorySetpoint(pos, vel);
}

std::optional<px4_msgs::msg::VehicleOdometry> Pelican::requestNEDOdometry() {
    return this->tac_core_.getNEDOdometry();
}

std::optional<px4_msgs::msg::VehicleStatus> Pelican::requestStatus() {
    return this->tac_core_.getStatus();
}

bool Pelican::commenceWaitForCommanderAck(uint16_t command) {
    return this->tac_core_.waitForCommanderAck(command);
}

void Pelican::commenceStopTacMapService() {
    this->tac_core_.stopService();
}

bool Pelican::commenceCheckOffboardEngagement() {
    return this->tac_core_.checkOffboardEngagement();
}

/************** From external modules, concluding here *************/
void Pelican::commenceFollowerOperations() {
    this->sendLogDebug("Received signal for transitioning to Follower");
    this->becomeFollower();
}

void Pelican::commenceLeaderOperations() {
    this->sendLogDebug("Received signal for transitioning to Leader");
    this->becomeLeader();
}

void Pelican::commenceCandidateOperations() {
    this->sendLogDebug("Received signal for transitioning to Candidate");
    this->becomeCandidate();
}

void Pelican::commenceIncreaseCurrentTerm() {
    this->setTerm(this->getCurrentTerm() + 1);
}

void Pelican::commenceSetTerm(uint64_t term) {
    this->setTerm(term);
}

void Pelican::commenceHeightCompensation(double odom_height) {
    auto maybe_target = this->getSetpointPosition();
    double target_height = this->getActualTargetHeight();

    if (this->commenceCheckOffboardEngagement() && maybe_target) {
        auto current_target = maybe_target.value();
        auto compensated_height = current_target.z + (target_height - odom_height);
        this->sendLogDebug(
            "Height| current: {:.4f}, target: {:.4f}, odom: {:.4f}, compensated: {:.4f}",
            current_target.z, target_height, odom_height, compensated_height
        );
        this->setSetpointPosition(current_target.set__z(compensated_height));
    }
}

void Pelican::commenceSharePosition(geometry_msgs::msg::Point pos) {
    this->sharePosition(pos);
}

void Pelican::commenceSetSetpointPosition(geometry_msgs::msg::Point p) {
    this->setSetpointPosition(p);
}

void Pelican::commenceSetSetpointVelocity(geometry_msgs::msg::Point v) {
    this->setSetpointVelocity(v);
}
