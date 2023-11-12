#include "PelicanModule/pelican.hpp"

// As a structure, I've decided no direct communications among modules
// is to be done. Everything passes though the main module and is redirected
// to the appropriate one

/*********************** To Heartbeat Module ***********************/
heartbeat Pelican::requestLastHb() {
    this->sendLogDebug("Received request for last heartbeat queued");
    return this->hb_core_.getLastHb();
}

int Pelican::requestNumberOfHbs() {
    this->sendLogDebug("Received request for number of heartbeats");
    return this->hb_core_.getNumberOfHbs();
}

void Pelican::commenceStopHeartbeatService() {
    this->hb_core_.stopService();
}

void Pelican::commenceStopElectionService() {
    this->el_core_.stopService();
}

void Pelican::commenceStopTacMapService() {
    this->tac_core_.stopService();
}

void Pelican::commenceStopUNSCService() {
    this->unsc_core_.stopService();
}

/*********************** To Election Module ************************/
void Pelican::commenceSetElectionStatus(int i) {
    this->sendLogDebug("Requesting setElectionStatus");
    // If there's some problem with the Election Module,
    // the code will just throw an error and terminate
    this->el_core_.setElectionStatus(i);
}

void Pelican::commenceResetElectionTimer() {
    this->sendLogDebug("Requesting resetElectionTimer");
    // If there's some problem with the Election Module,
    // the code will just throw an error and terminate
    this->el_core_.resetElectionTimer();
}

/************************* To UNSC Module **************************/
void Pelican::commencePublishVehicleCommand(
    uint16_t command, float param1, float param2, float param3, float param4, float param5,
    float param6, float param7
) {
    this->sendLogDebug("Requesting publishVehicleCommand");
    this->tac_core_.publishVehicleCommand(
        command, param1, param2, param3, param4, param5, param6, param7
    );
}

void Pelican::commencePublishOffboardControlMode() {
    this->sendLogDebug("Requesting publishOffboardControlMode");
    this->tac_core_.publishOffboardControlMode();
}

void Pelican::commencePublishTrajectorySetpoint(float x, float y, float z, float yaw) {
    this->sendLogDebug("Requesting publishTrajectorySetpoint");
    this->tac_core_.publishTrajectorySetpoint(x, y, z, yaw);
}

std::optional<px4_msgs::msg::VehicleGlobalPosition> Pelican::requestGlobalPosition() {
    this->sendLogDebug("Requesting global position");
    return this->tac_core_.getGlobalPosition();
}

std::optional<px4_msgs::msg::VehicleOdometry> Pelican::requestOdometry() {
    this->sendLogDebug("Requesting odometry");
    return this->tac_core_.getOdometry();
}

std::optional<px4_msgs::msg::VehicleCommandAck> Pelican::requestAck() {
    this->sendLogDebug("Requesting ack");
    return this->tac_core_.getAck();
}

std::optional<px4_msgs::msg::VehicleStatus> Pelican::requestStatus() {
    this->sendLogDebug("Requesting status");
    return this->tac_core_.getStatus();
}
