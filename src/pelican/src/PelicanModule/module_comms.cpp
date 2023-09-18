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