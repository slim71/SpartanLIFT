#include "ElectionModule/election.hpp"
#include "PelicanModule/pelican.hpp"

bool ElectionModule::isElectionCompleted() const {
    std::lock_guard lock(this->election_completed_mutex_);
    return this->election_completed_;
}

bool ElectionModule::isVotingCompleted() const {
    std::lock_guard lock(this->voting_completed_mutex_);
    return this->voting_completed_;
}

bool ElectionModule::checkForExternalLeader() {
    if (!this->isExternalLeaderElected()) {
        return false;
    }

    if (this->gatherNumberOfHbs() == 0) {
        // Standard behavior: boolean is more important than the heartbeat
        // This is equal to "no heartbeat received YET, but external leader is there"
        return true;
    }

    auto last_hb_received = this->gatherLastHb();

    if (last_hb_received.term >= this->gatherCurrentTerm()) {
        return true;
    } else {
        // Reject external leader and continue, as no heartbeat arrived
        this->sendLogDebug("Most recent heartbeat has old term");
        this->clearElectionStatus();
        return false;
    }
}

bool ElectionModule::isExternalLeaderElected() const {
    std::lock_guard lock(this->external_leader_mutex_);
    return this->external_leader_elected_;
}

bool ElectionModule::isLeaderElected() const {
    std::lock_guard lock(this->leader_mutex_);
    return this->leader_elected_;
}
