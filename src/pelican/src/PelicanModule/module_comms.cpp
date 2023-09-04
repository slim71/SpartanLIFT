#include "PelicanModule/pelican.hpp"

// As a structure, I've decided no direct communications among modules
// is to be done. Everything passes though the main module and is redirected
// to the appropriate one


/*********************** To Heartbeat Module ***********************/
heartbeat Pelican::requestLastHb() {
    return this->hb_core_.getLastHb();
}

int Pelican::requestNumberOfHbs() {
    return this->hb_core_.getNumberOfHbs();
}

/*********************** To Election Module ************************/
void Pelican::requestSetElectionStatus(int i) {
    // If there's some problem with the Election Module,
    // the code will just throw an error and terminate
    this->el_core_.setElectionStatus(i);
}

void Pelican::requestResetElectionTimer() {
    // If there's some problem with the Election Module,
    // the code will just throw an error and terminate
    this->el_core_.resetElectionTimer();
}