#include "pelican.hpp"

void Pelican::sendHeartbeat() const {
    this->logInfo("Sending heartbeat");

    comms::msg::Heartbeat hb;
    hb.term_id = this->getCurrentTerm();
    hb.leader_id = this->getID();
    hb.timestamp = this->now(); // rclcpp::Clock{RCL_SYSTEM_TIME}.now()

    if (this->pub_to_heartbeat_topic_) {
        this->pub_to_heartbeat_topic_->publish(hb);
    } else {
        this->logError("Publisher to heartbeat topic is not defined!");
    }
}

void Pelican::stopHeartbeat() {
    this->hb_transmission_timer_->cancel();
}

void Pelican::storeHeartbeat(const comms::msg::Heartbeat msg) {
    if (msg.term_id < this->getCurrentTerm()) {
        // Ignore heartbeat
        this->logWarning(
            "Ignoring heartbeat received with previous term ID ({} vs {})", msg.term_id,
            this->getCurrentTerm()
        );
        return;
    }

    // If it comes to this, the msg.term is at least equal to the node's current term

    // For any node; this should not apply to leaders
    this->setElectionStatus(msg.leader_id);

    this->logDebug("Resetting election_timer_...");
    this->resetTimer(this->election_timer_
    ); // Reset the election_timer_, used to be sure there's a leader, to election_timeout
    this->logDebug(
        "After resetting, timer is {} ms", this->election_timer_->time_until_trigger().count() / 10
    );

    // Keep heartbeat vector limited
    if (this->getNumberOfHbs() >= this->getMaxHbs()) {
        this->flushHeartbeats();
    }

    heartbeat hb;
    hb.term = msg.term_id;
    hb.leader = msg.leader_id;
    hb.timestamp = msg.timestamp;

    this->logInfo("Received heartbeat from agent {} during term {}", msg.leader_id, msg.term_id);

    this->hbs_mutex_.lock();
    this->received_hbs_.push_back(hb);
    // Sort such that older hb is first, to be sure
    std::sort(
        this->received_hbs_.begin(), this->received_hbs_.end(),
        [](const heartbeat& a, const heartbeat& b) { return a.timestamp < b.timestamp; }
    );
    this->hbs_mutex_.unlock();

    if (this->getRole() == leader) { // Switch back to follower!
        // This should never be needed, since Raft guarantees safety
        this->logWarning("As a leader, I've received a heartbeat from some other leader agent");
        this->becomeFollower();
    }
}

void Pelican::flushHeartbeats() {
    std::lock_guard<std::mutex> lock(this->hbs_mutex_);
    this->received_hbs_.clear();
}
