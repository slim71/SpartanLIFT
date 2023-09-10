#include "ElectionModule/election.hpp"
#include "types.hpp"
#include "utilities.hpp"

/************************** Public methods *************************/
void ElectionModule::resetElectionTimer() {
    // Reset the election_timer_, used to be sure there's a leader, to election_timeout
    resetTimer(this->election_timer_);
    if(this->election_timer_)
        this->sendLogDebug(
            "After resetting, timer is {} ms", this->election_timer_->time_until_trigger().count() / 10
        );
}

void ElectionModule::resetSubscriptions() {
    // This is not needed by leaders, since Raft guarantees safety;
    // it's just an additional check to avoid multiple leaders
    resetSharedPointer(this->sub_to_leader_election_topic_);
    resetSharedPointer(this->sub_to_request_vote_rpc_topic_);
}

void ElectionModule::setElectionStatus(int id) {
    this->setExternalLeaderElected();
    this->setLeaderElected();
    this->setLeader(id);
}

/************************* Private methods *************************/
void ElectionModule::resetVotingWindow() {
    this->unsetVotingCompleted();
    this->setRandomBallotWaittime();
}

void ElectionModule::setElectionCompleted() {
    this->voting_timer_->cancel();
    std::lock_guard<std::mutex> lock(this->election_completed_mutex_);
    this->election_completed_ = true;
}

void ElectionModule::unsetElectionCompleted() {
    std::lock_guard<std::mutex> lock(this->election_completed_mutex_);
    this->election_completed_ = false;
}

void ElectionModule::setVotingCompleted() {
    std::lock_guard<std::mutex> lock(this->voting_completed_mutex_);
    this->voting_completed_ = true;
}

void ElectionModule::unsetVotingCompleted() {
    std::lock_guard<std::mutex> lock(this->voting_completed_mutex_);
    this->voting_completed_ = false;
}

void ElectionModule::setRandomBallotWaittime() {
    this->new_ballot_waittime_ =
        std::chrono::milliseconds {this->random_distribution_(this->random_engine_)};
    this->sendLogDebug("new_ballot_waittime_ set to {}", this->new_ballot_waittime_.count());
}

void ElectionModule::setRandomElectionTimeout() {
    this->election_timeout_ =
        std::chrono::milliseconds {this->random_distribution_(this->random_engine_)};
    this->sendLogDebug("election_timeout_ set to {}", this->election_timeout_.count());
}

void ElectionModule::clearElectionStatus() {
    this->unsetExternalLeaderElected();
    this->unsetLeaderElected();
    this->setLeader();
}

void ElectionModule::setIsTerminated() {
    std::lock_guard<std::mutex> lock(this->terminated_mutex_);
    this->is_terminated_ = true;
}

void ElectionModule::setExternalLeaderElected() {
    std::lock_guard<std::mutex> lock(this->external_leader_mutex_);
    this->external_leader_elected_ = true;
}

void ElectionModule::unsetExternalLeaderElected() {
    std::lock_guard<std::mutex> lock(this->external_leader_mutex_);
    this->external_leader_elected_ = false;
}

void ElectionModule::setLeader(int id) {
    this->setLeaderElected();

    // id=0 means election is not finished or has been reset
    if (id >= 0) {
        this->leader_id_ = id;
    } else {
        this->sendLogWarning("Tried to setup a negative ID! Setting it at -1 as indication...");
        this->leader_id_ = -1;
    }
}

void ElectionModule::setLeaderElected() {
    std::lock_guard<std::mutex> lock(this->leader_mutex_);
    this->leader_elected_ = true;
}

void ElectionModule::unsetLeaderElected() {
    std::lock_guard<std::mutex> lock(this->leader_mutex_);
    this->leader_elected_ = false;
}
