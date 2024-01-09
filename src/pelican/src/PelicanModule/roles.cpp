#include "LoggerModule/logger.hpp"
#include "PelicanModule/pelican.hpp"

/************** Actions initiated from outside the module *************/
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

/************************** Public methods ***************************/
bool Pelican::isLeader() const {
    return (this->getRole() == leader);
}

bool Pelican::isFollower() const {
    return (this->getRole() == follower);
}

bool Pelican::isCandidate() const {
    return (this->getRole() == candidate);
}

/************************** Private methods ***************************/
void Pelican::becomeLeader() {
    this->setRole(leader);
    this->logger_.cacheRole(leader);
    this->sendLogInfo("Becoming {}", roles_to_string(leader));

    this->hb_core_.flushHeartbeats();
    this->el_core_.flushVotes();

    // Topic preparations outside specific actions
    this->hb_core_.resetSubscription();
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

void Pelican::rogerWillCo(
    const std::shared_ptr<comms::srv::FleetInfoExchange::Request> request,
    const std::shared_ptr<comms::srv::FleetInfoExchange::Response> response
) {
    response->leader_id = this->id_;
    response->ack = true;

    if (request->presence) {
        this->sendLogDebug("Notifying I'm the leader to the user!");
        response->present = true;
        response->taking_off = false;
        response->landing = false;
    }

    if (request->takeoff) {
        // TODO: send command to other agents
        this->sendLogDebug("Notifying start of takeoff!");
        response->present = false;
        response->taking_off = true;
        response->landing = false;
    }

    if (request->landing) {
        // TODO: send command to other agents
        this->sendLogDebug("Notifying start of landing operations!");
        response->present = false;
        response->taking_off = false;
        response->landing = true;
    }
}
