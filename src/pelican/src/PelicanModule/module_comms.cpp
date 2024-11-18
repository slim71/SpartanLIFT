/**
 * @file module_comms.cpp
 * @author Simone Vollaro (slim71sv@gmail.com)
 * @brief Methods dealing with data and functionality exchange among modules.
 * @version 1.0.0
 * @date 2024-11-17
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "PelicanModule/pelican.hpp"

/************************* Heartbeat Module ************************/
/**
 * @brief Forward request of the last heartbeat to the heartbeat module.
 *
 * @return heartbeat Last received heartbeat
 */
heartbeat Pelican::requestLastHb() const {
    return this->hb_core_.getLastHb();
}

/**
 * @brief Forward the request of the number of received heartbeats to the heartbeat module.
 *
 * @return int Number of heartbeat received.
 */
int Pelican::requestNumberOfHbs() const {
    return this->hb_core_.getNumberOfHbs();
}

/**
 * @brief Forward the stop signal to the heartbeat module.
 *
 */
void Pelican::commenceStopHeartbeatService() {
    this->hb_core_.stopService();
}

/**
 * @brief Forward the interruption of heartbeat transmission the heartbeat module.
 *
 */
void Pelican::commenceStopHeartbeatTransmission() {
    this->hb_core_.stopHeartbeatTransmission();
}

/************************* Election Module *************************/
/**
 * @brief Forward the election status to the election module.
 *
 * @param i ID of the newly elected leader.
 */
void Pelican::commenceSetElectionStatus(int i) {
    this->el_core_.setElectionStatus(i);
}

/**
 * @brief Forward the reset of the election timer to the election module.
 *
 */
void Pelican::commenceResetElectionTimer() {
    this->el_core_.resetElectionTimer();
}

/**
 * @brief Forward the stop signal to the election module.
 *
 */
void Pelican::commenceStopElectionService() {
    this->el_core_.stopService();
}

/**
 * @brief Forward the request of the leader ID to the election module.
 *
 * @return unsigned int Leader ID.
 */
unsigned int Pelican::requestLeaderID() const {
    return this->el_core_.getLeaderID();
}

/**************************** UNSC Module *****************************/
/**
 * @brief Forward the arm trigger to the UNSC module and returns its success.
 *
 * @return true
 * @return false
 */
bool Pelican::initiateArm() {
    return this->unsc_core_.arm();
}

/**
 * @brief Forward the "set home" trigger to the UNSC module and return its success.
 *
 * @return true
 * @return false
 */
bool Pelican::initiateSetHome() {
    return this->unsc_core_.setHome();
}

/**
 * @brief Forward the takeoff trigger to the UNSC module and return its success.
 *
 * @return true
 * @return false
 */
bool Pelican::initiateTakeoff() {
    return this->unsc_core_.takeoff();
}

/**
 * @brief Forward the land trigger to the UNSC module and return its success.
 *
 * @return true
 * @return false
 */
bool Pelican::initiateLand() {
    return this->unsc_core_.land();
}

/**
 * @brief Forward the "return to launch position" trigger to the UNSC module and return its success.
 *
 * @return true
 * @return false
 */
bool Pelican::initiateReturnToLaunchPosition() {
    return this->unsc_core_.returnToLaunchPosition();
}

/**
 * @brief Forward the consesus trigger to the USC module.
 *
 */
void Pelican::initiateConsensus() {
    this->unsc_core_.activateConsensus();
}

/**
 * @brief Forward the formation actions trigger to the UNSC module.
 *
 */
void Pelican::initiateFormationActions() {
    this->unsc_core_.formationActions();
}

/**
 * @brief Forward the request of the position setpoint to the UNSC module.
 *
 * @return std::optional<geometry_msgs::msg::Point> Position setpoint, if available.
 */
std::optional<geometry_msgs::msg::Point> Pelican::initiateGetPositionSetpoint() const {
    return this->unsc_core_.getPositionSetpoint();
}

/**
 * @brief Forward the request of the target position to the UNSC module.
 *
 * @param p Target position.
 */
void Pelican::initiateSetTargetPosition(geometry_msgs::msg::Point p) {
    this->unsc_core_.setTargetPosition(p);
}

/**
 * @brief Forward the setting of a new position setpoint to the UNSC module.
 *
 * @param p Position setpoint to set.
 */
void Pelican::initiateSetPositionSetpoint(geometry_msgs::msg::Point p) {
    this->unsc_core_.setPositionSetpoint(p);
}

/**
 * @brief Forward the unblocking of the formation to the UNSC module.
 *
 */
void Pelican::initiateUnblockFormation() {
    this->unsc_core_.unblockFormation();
}

/**
 * @brief Forward the setting of a new target height to the UNSC module.
 *
 * @param height Target height to follow.
 */
void Pelican::commenceSetActualTargetHeight(double height) {
    this->unsc_core_.setActualTargetHeight(height);
}

/**
 * @brief Forward the request of the target height to the UNSC module.
 *
 * @return double Target height stored.
 */
double Pelican::requestActualTargetHeight() const {
    return this->unsc_core_.getActualTargetHeight();
}

/**
 * @brief Forward the request of the target position to the UNSC module.
 *
 * @return std::optional<geometry_msgs::msg::Point> Target position, if available.
 */
std::optional<geometry_msgs::msg::Point> Pelican::requestTargetPosition() const {
    return this->unsc_core_.getTargetPosition();
}

/**
 * @brief Forward the request of the flag related to the simulation status to the UNSC module.
 *
 * @return true
 * @return false
 */
bool Pelican::requestSimulationReady() {
    return this->unsc_core_.getSimulationReady();
}

/**
 * @brief Forward the stop signal to the UNSC module.
 *
 */
void Pelican::commenceStopUNSCService() {
    this->unsc_core_.stopService();
}

/**
 * @brief Forward the trigger of the height compensation to the UNSC module.
 *
 * @param odom_height Reference height for the compensation.
 */
void Pelican::commenceHeightCompensation(double odom_height) {
    this->unsc_core_.heightCompensation(odom_height);
}

/*************************** TacMap Module ****************************/
/**
 * @brief Forward the vehicle command to publish, along with its parameters, to the TacMap module.
 *
 * @param command
 * @param param1
 * @param param2
 * @param param3
 * @param param4
 * @param param5
 * @param param6
 * @param param7
 */
void Pelican::commencePublishVehicleCommand(
    uint16_t command, float param1, float param2, float param3, float param4, float param5,
    float param6, float param7
) {
    this->tac_core_.publishVehicleCommand(
        command, param1, param2, param3, param4, param5, param6, param7
    );
}

/**
 * @brief Forward the switch to the offboard mode to the TacMap module.
 *
 */
void Pelican::commencePublishOffboardControlMode() {
    this->tac_core_.publishOffboardControlMode();
}

/**
 * @brief Forward the trajectory setpoint to publish to the TacMap module.
 *
 * @param pos Position setpoint to publish.
 * @param vel Velocity setpoint to publish.
 */
void Pelican::commencePublishTrajectorySetpoint(
    geometry_msgs::msg::Point pos, geometry_msgs::msg::Point vel
) {
    this->tac_core_.publishTrajectorySetpoint(pos, vel);
}

/**
 * @brief Forward the start of the ack wait period for the specified command to the TacMap module.
 *
 * @param command Command of which to wait an ack.
 * @return true If an ACK is received.
 * @return false If a NACK is received.
 */
bool Pelican::commenceWaitForCommanderAck(uint16_t command) {
    return this->tac_core_.waitForCommanderAck(command);
}

/**
 * @brief Forward the stop signal to the TacMap module.
 *
 */
void Pelican::commenceStopTacMapService() {
    this->tac_core_.stopService();
}

/**
 * @brief Forward the check for the offboard engagement to the TacMap module.
 *
 * @return true
 * @return false
 */
bool Pelican::commenceCheckOffboardEngagement() {
    return this->tac_core_.checkOffboardEngagement();
}

/**
 * @brief Forward the request of ENU odometry data to the TacMap module.
 *
 * @return std::optional<nav_msgs::msg::Odometry> ENU odometry data, if available.
 */
std::optional<nav_msgs::msg::Odometry> Pelican::requestENUOdometry() const {
    return this->tac_core_.getENUOdometry();
}

/**
 * @brief Forward the request of status data to the TacMap module.
 *
 * @return std::optional<px4_msgs::msg::VehicleStatus> Status data, if available.
 */
std::optional<px4_msgs::msg::VehicleStatus> Pelican::requestStatus() const {
    return this->tac_core_.getStatus();
}

/************** From external modules, concluding here *************/
/**
 * @brief Start follower operations on request.
 *
 */
void Pelican::commenceFollowerOperations() {
    this->becomeFollower();
}

/**
 * @brief Start leader operations on request.
 *
 */
void Pelican::commenceLeaderOperations() {
    this->becomeLeader();
}

/**
 * @brief Start candidate operations on request.
 *
 */
void Pelican::commenceCandidateOperations() {
    this->becomeCandidate();
}

/**
 * @brief Start the increase of the term ID on request.
 *
 */
void Pelican::commenceIncreaseCurrentTerm() {
    this->setTerm(this->getCurrentTerm() + 1);
}

/**
 * @brief Start the setting of a new term ID on request.
 *
 * @param term Term ID to set.
 */
void Pelican::commenceSetTerm(uint64_t term) {
    this->setTerm(term);
}

/**
 * @brief Start a position sharing on request.
 *
 * @param pos Position to share.
 */
void Pelican::commenceSharePosition(geometry_msgs::msg::Point pos) {
    this->sharePosition(pos);
}

/**
 * @brief Start the notification of agent being in formation on request.
 *
 */
void Pelican::commenceNotifyAgentInFormation() {
    this->notifyAgentInFormation();
}

/**
 * @brief Start the synchronization after an operation completion on request.
 *
 * @param cmd Command to check.
 */
void Pelican::commenceSyncCompletedOp(uint32_t cmd) {
    this->syncCompletedOp(cmd);
}

// Map with pairs: agent ID - agent desired position
/**
 * @brief Start the transmission of the formation positions on request.
 *
 * @param positions Formation positions to send.
 */
void Pelican::commenceSendDesiredFormationPositions(
    std::unordered_map<unsigned int, geometry_msgs::msg::Point> positions
) {
    this->sendDesiredFormationPositions(positions);
}

/**
 * @brief Start the request of desired position to the neighbor on request.
 *
 * @param id Neighbor to which to request the desired position.
 */
void Pelican::commenceAskDesPosToNeighbor(unsigned int id) {
    this->askDesPosToNeighbor(id);
}

/**
 * @brief Start the cargo attachment/detachment on request.
 *
 * @param attach Boolean stating whether to attach or detach the cargo.
 */
void Pelican::commenceCargoAttachment(bool attach) {
    this->cargoAttachment(attach);
}

/**
 * @brief Start the reset of the carrying status on request.
 *
 */
void Pelican::commenceUnsetCarryingStatus() {
    this->unsetCarryingStatus();
}

/**
 * @brief Start the reset of the formation status on request.
 *
 */
void Pelican::commenceUnsetFormationAchieved() {
    this->unsetFormationAchieved();
}

/**
 * @brief Start the reset of the array containing the agents in formation on request.
 *
 */
void Pelican::commenceEmptyFormationResults() {
    this->emptyFormationResults();
}

/**
 * @brief Start the storage of a newly discovered agent on request.
 *
 * @param id Agent discovered.
 */
void Pelican::commenceStoreAttendance(unsigned int id) {
    this->storeAttendance(id);
}
