#include "pelican.hpp"

void PelicanUnit::sendHeartbeat() {
    this->logInfo("Sending heartbeat");

    comms::msg::Heartbeat hb;
    hb.term_id = this->getCurrentTerm();
    hb.leader_id = this->getID();
    hb.timestamp = this->now(); // rclcpp::Clock{RCL_SYSTEM_TIME}.now()

    this->pub_to_heartbeat_topic_->publish(hb);

}

void PelicanUnit::stopHeartbeat() {
    this->hb_transmission_timer_->cancel();
}

// void PelicanUnit::checkHeartbeat() {
//     this->hbs_mutex_.lock();
//     // Missing heartbeats from leader
//     if (!this->received_hbs_.empty()) { // Leader is sending heartbearts

//         this->received_hbs_.pop(); // TODO: needed?
//         this->hbs_mutex_.unlock();
//         this->logInfo("Heartbeat received");
//     }
// }

void PelicanUnit::storeHeartbeat(const comms::msg::Heartbeat msg) {
    // TODO: refine for leader id and term id
    // TODO: consider the case of changing follower ID in two subsequent hbs?

    // First of all, check if it's too late
    if (this->isFollower() && this->checkElectionTimedOut()) {
        this->logInfo("No heartbeat received within the 'election_timeout' window; switching to candidate...");
        // TODO: delete if deleting checkHeartbeat
        this->hb_monitoring_timer_->cancel(); // Cancel the wall_timer for further checking of heartbeats

        this->election_timer_->cancel();

        this->becomeCandidate(); // transition to Candidate state
        return;
    }

    if (msg.term_id < this->getCurrentTerm()) {
        // Ignore heartbeat
        this->logDebug("Ignoring heartbeat received with previous term ID");
        return;
    }

    heartbeat hb;
    hb.term = msg.term_id;
    hb.leader = msg.leader_id;
    hb.timestamp = msg.timestamp;

    this->hbs_mutex_.lock();
    this->received_hbs_.push_back(hb);
    // Sort such that older hb is first, to be sure
    std::sort(this->received_hbs_.begin(),
              this->received_hbs_.end(),
              [](const heartbeat &a, const heartbeat &b) {
                return a.timestamp < b.timestamp;
              });
    this->hbs_mutex_.unlock();
        
    this->logDebug("Resetting election_timer_...");
    this->election_timer_->reset(); // Reset the election_timer_, used to be sure there's a leader, to election_timeout
    this->logDebug("After resetting, timer is {} ms", this->election_timer_->time_until_trigger().count()/10);

    if (this->getRole() == candidate) {
        this->setExternalLeaderElected();
    }
}