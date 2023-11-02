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

void Pelican::commenceStopHeartbeat() {
    this->hb_core_.stopHeartbeat();
}

void Pelican::commenceStopBallotThread() {
    this->el_core_.stopBallotThread();
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
    // If there's some problem with the Election Module,
    // the code will just throw an error and terminate
    this->tac_core_.publishVehicleCommand(
        command, param1, param2, param3, param4, param5, param6, param7
    );
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
