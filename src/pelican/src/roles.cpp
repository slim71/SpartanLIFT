#include "pelican.hpp"
#include "logger.hpp"
#include "types.hpp"
#include "utilities.hpp"

void Pelican::becomeLeader() {
    this->setRole(leader);
    this->sendLogInfo("Becoming {}", roles_to_string(this->getRole()));
    this->hb_core_.flushStorage();
    // TODO: flushVotes?

    // Unsubscribe from topics
    this->hb_core_.resetSubscription();
    this->el_core_.resetSubscriptions();

    this->hb_core_.setupPublisher();

    this->hb_core_.sendNow(); // To promptly notify all agents about the new leader
    this->hb_core_.setTransmissionTimer();
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

void Pelican::commenceFollowerOperations() {
    this->becomeFollower();
}

void Pelican::commenceLeaderOperations() {
    this->becomeLeader();
}

void Pelican::commenceCandidateOperations() {
    this->becomeCandidate();
}

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
