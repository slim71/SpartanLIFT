#include "PelicanModule/pelican.hpp"
#include "LoggerModule/logger.hpp"
#include "types.hpp"
#include "utilities.hpp"

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
    this->sendLogDebug("Agent is leader? {}", this->getRole() == leader);
    return (this->getRole() == leader);
}

bool Pelican::isFollower() const {
    this->sendLogDebug("Agent is follower? {}", this->getRole() == follower);
    return (this->getRole() == follower);
}

bool Pelican::isCandidate() const {
    this->sendLogDebug("Agent is candidate? {}", this->getRole() == candidate);
    return (this->getRole() == candidate);
}

/************************** Private methods ***************************/
void Pelican::becomeLeader() {
    this->setRole(leader);
    this->sendLogInfo("Becoming {}", roles_to_string(this->getRole()));
    this->hb_core_.flushHeartbeats();
    this->el_core_.flushVotes();

    // Unsubscribe from topics
    this->hb_core_.resetSubscription();
    this->el_core_.resetSubscriptions();

    this->hb_core_.setupPublisher();

    this->hb_core_.sendNow(); // To promptly notify all agents about the new leader
    this->hb_core_.setupTransmissionTimer();
}

void Pelican::becomeFollower() {
    this->setRole(follower);
    this->sendLogInfo("Becoming {}", roles_to_string(this->getRole()));
    this->el_core_.prepareTopics();

    this->el_core_.followerActions();
}

void Pelican::becomeCandidate() {
    this->setRole(candidate);
    this->sendLogInfo("Becoming {}", roles_to_string(this->getRole()));

    this->el_core_.prepareTopics();
    this->hb_core_.setupSubscription();

    this->el_core_.prepareForCandidateActions();
    this->el_core_.candidateActions();
}
