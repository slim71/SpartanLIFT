#include "pelican.hpp"

void PelicanUnit::sendHeartbeat() {
    RCLCPP_INFO(get_logger(), "Sending heartbeat");

    comms::msg::Heartbeat hb;
    hb.term_id = this->current_term_;
    hb.leader_id = this->id_;
    hb.timestamp = this->now(); // rclcpp::Clock{RCL_SYSTEM_TIME}.now()

    this->pub_to_heartbeat_topic_->publish(hb);

}

void PelicanUnit::stopHeartbeat() {
    this->hb_transmission_timer_->cancel();
}

void PelicanUnit::checkHeartbeat() {
    // Cannot use the lock_guard here because the first condition calls another function without changing scope
    this->hbs_mutex_.lock();
    if (this->received_hbs_.empty()) {
        this->hbs_mutex_.unlock();
        RCLCPP_INFO(get_logger(), "No heartbeat received; switching to candidate...");
        this->hb_monitoring_timer_->cancel();
        this->current_term_++; // increment term ID
        this->becomeCandidate(); // transition to Candidate state
    } else {
        // TODO: check for the hb term and timestamp to be sure?
        // TODO: consider the case of changing follower ID in two subsequent hbs?
        RCLCPP_INFO(get_logger(), "Heartbeat received");
        this->received_hbs_.pop();
        this->hbs_mutex_.unlock();
    }
}

void PelicanUnit::storeHeartbeat(const comms::msg::Heartbeat msg) {
    heartbeat hb;
    hb.term = msg.term_id;
    hb.leader = msg.leader_id;
    hb.timestamp = msg.timestamp;

    // Ensure safe access to received_hbs_
    std::lock_guard<std::mutex> lock(this->hbs_mutex_);
    this->received_hbs_.push(hb);    
}