#include "ElectionModule/election.hpp"
#include "pelican.hpp"

bool ElectionModule::checkElectionCompleted() const {
    // Ensure safe access to election_timed_out
    std::lock_guard<std::mutex> lock(this->election_completed_mutex_);
    return this->election_completed_;
}

bool ElectionModule::checkVotingCompleted() const {
    std::lock_guard<std::mutex> lock(this->voting_completed_mutex_);
    return this->voting_completed_;
}

bool ElectionModule::checkIsTerminated() const {
    std::lock_guard<std::mutex> lock(this->terminated_mutex_);
    return this->is_terminated_;
}

bool ElectionModule::checkForExternalLeader() {
    // If the (external) leader’s term is at least as large as the candidate’s current term,
    // then the candidate recognizes the leader as legitimate and returns to follower state

    if (!this->checkExternalLeaderElected()) {
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
        // reject external leader and continue as no heartbeat arrived
        this->sendLogDebug("Most recent heartbeat has old term");
        this->clearElectionStatus();
        return false;
    }
}

bool ElectionModule::checkExternalLeaderElected() const {
    std::lock_guard<std::mutex> lock(this->external_leader_mutex_);
    return this->external_leader_elected_;
}

bool ElectionModule::checkLeaderElected() const {
    std::lock_guard<std::mutex> lock(this->leader_mutex_);
    return this->leader_elected_;
}
