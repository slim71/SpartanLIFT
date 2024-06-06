#include "PelicanModule/pelican.hpp"

/************************* Heartbeat Module ************************/
heartbeat Pelican::requestLastHb() const {
    return this->hb_core_.getLastHb();
}

int Pelican::requestNumberOfHbs() const {
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

std::optional<geometry_msgs::msg::Point> Pelican::initiateGetSetpointPosition() const {
    return this->unsc_core_.getSetpointPosition();
}

void Pelican::initiateSetTargetPosition(geometry_msgs::msg::Point p) {
    this->unsc_core_.setTargetPosition(p);
}

void Pelican::initiateSetSetpointPosition(geometry_msgs::msg::Point p) {
    this->unsc_core_.setSetpointPosition(p);
}

void Pelican::commenceSetActualTargetHeight(double height) {
    this->unsc_core_.setActualTargetHeight(height);
}

double Pelican::requestActualTargetHeight() const {
    return this->unsc_core_.getActualTargetHeight();
}

std::optional<geometry_msgs::msg::Point> Pelican::requestTargetPosition() const {
    return this->unsc_core_.getTargetPosition();
}

void Pelican::commenceStopUNSCService() {
    this->unsc_core_.stopService();
}

void Pelican::commenceHeightCompensation(double odom_height) {
    this->unsc_core_.heightCompensation(odom_height);
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

bool Pelican::commenceWaitForCommanderAck(uint16_t command) {
    return this->tac_core_.waitForCommanderAck(command);
}

void Pelican::commenceStopTacMapService() {
    this->tac_core_.stopService();
}

bool Pelican::commenceCheckOffboardEngagement() {
    return this->tac_core_.checkOffboardEngagement();
}

std::optional<nav_msgs::msg::Odometry> Pelican::requestENUOdometry() const {
    return this->tac_core_.getENUOdometry();
}

std::optional<px4_msgs::msg::VehicleStatus> Pelican::requestStatus() const {
    return this->tac_core_.getStatus();
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

void Pelican::commenceSharePosition(geometry_msgs::msg::Point pos) {
    this->sharePosition(pos);
}
