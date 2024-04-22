#include "LoggerModule/logger.hpp"
#include "PelicanModule/pelican.hpp"

/*************************** Role handling ****************************/
void Pelican::becomeLeader() {
    if (this->role_ == leader)
        return;

    this->setRole(leader);
    this->sendLogInfo("Becoming {}", roles_to_string(leader));

    this->hb_core_.flushHeartbeats();
    this->el_core_.flushVotes();

    // Topic preparations
    this->el_core_.resetSubscriptions();
    this->hb_core_.setupPublisher();

    // Service preparation
    resetSharedPointer(this->fleetinfo_client_);
    this->teleopdata_server_ = this->create_service<comms::srv::TeleopData>(
        "contactLeader_service",
        std::bind(&Pelican::rogerWillCo, this, std::placeholders::_1, std::placeholders::_2)
    );
    this->fleetinfo_server_ = this->create_service<comms::srv::FleetInfo>(
        "shareinfo_service",
        std::bind(&Pelican::targetNotification, this, std::placeholders::_1, std::placeholders::_2)
    );

    this->hb_core_.sendNow(); // To promptly notify all agents about the new leader
    this->hb_core_.setupTransmissionTimer();
}

void Pelican::becomeFollower() {
    if (this->role_ == follower)
        return;

    this->setRole(follower);
    this->sendLogInfo("Becoming {}", roles_to_string(follower));

    // Topic preparations
    this->commenceStopHeartbeatService();
    this->hb_core_.resetPublisher();
    this->el_core_.prepareTopics();

    // Service preparation
    resetSharedPointer(this->teleopdata_server_);
    resetSharedPointer(this->fleetinfo_server_);
    this->fleetinfo_client_ = this->create_client<comms::srv::FleetInfo>("shareinfo_service");

    this->el_core_.followerActions();
}

void Pelican::becomeCandidate() {
    if (this->role_ == candidate)
        return;

    this->setRole(candidate);
    this->sendLogInfo("Becoming {}", roles_to_string(candidate));

    // Topic preparations
    this->el_core_.prepareTopics();
    this->commenceStopHeartbeatService();
    this->hb_core_.resetPublisher();
    this->hb_core_.setupSubscription();

    this->el_core_.candidateActions();
}
