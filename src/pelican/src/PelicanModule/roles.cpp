#include "LoggerModule/logger.hpp"
#include "PelicanModule/pelican.hpp"

/*************************** Role handling ****************************/
void Pelican::becomeLeader() {
    if (this->role_ == leader)
        return;

    this->setRole(leader);
    this->logger_.cacheRole(leader);
    this->sendLogInfo("Becoming {}", roles_to_string(leader));

    this->hb_core_.flushHeartbeats();
    this->el_core_.flushVotes();

    // Topic preparations outside specific actions
    this->el_core_.resetSubscriptions();
    this->hb_core_.setupPublisher();

    this->fleetinfo_server_ = this->create_service<comms::srv::FleetInfoExchange>(
        "contactLeader_service",
        std::bind(&Pelican::rogerWillCo, this, std::placeholders::_1, std::placeholders::_2)
    );

    this->hb_core_.sendNow(); // To promptly notify all agents about the new leader
    this->hb_core_.setupTransmissionTimer();
}

void Pelican::becomeFollower() {
    if (this->role_ == follower)
        return;

    this->setRole(follower);
    this->logger_.cacheRole(follower);
    this->sendLogInfo("Becoming {}", roles_to_string(follower));

    this->commenceStopHeartbeatService();
    this->hb_core_.resetPublisher();
    this->el_core_.prepareTopics();
    resetSharedPointer(this->fleetinfo_server_);

    this->el_core_.followerActions();
}

void Pelican::becomeCandidate() {
    if (this->role_ == candidate)
        return;

    this->setRole(candidate);
    this->logger_.cacheRole(candidate);
    this->sendLogInfo("Becoming {}", roles_to_string(candidate));

    // Topic preparations outside specific actions
    this->el_core_.prepareTopics();
    this->commenceStopHeartbeatService();
    this->hb_core_.resetPublisher();
    this->hb_core_.setupSubscription();

    this->el_core_.candidateActions();
}
