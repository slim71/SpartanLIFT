#include "pelican.hpp"

void PelicanUnit::becomeLeader() {
    RCLCPP_INFO(get_logger(), "Becoming Leader");
    this->role_ = leader;

    this->pub_to_heartbeat_topic_ = this->create_publisher<comms::msg::Heartbeat>(
                                                this->heartbeat_topic_, 10
                                            ); // TODO: 10 (or better value) in constant

    std::chrono::milliseconds sleep_time {1000}; // TODO: value?

    this->hb_transmission_timer_ = this->create_wall_timer(sleep_time, std::bind(&PelicanUnit::sendHeartbeat, this), this->exclusive_group_);
}

void PelicanUnit::becomeFollower() {
    RCLCPP_INFO(get_logger(), "Becoming Follower");
    this->role_ = follower;

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    auto reentrant_opt = rclcpp::SubscriptionOptions();

    reentrant_opt.callback_group = this->reentrant_group_;
    this->sub_to_heartbeat_topic_ = this->create_subscription<comms::msg::Heartbeat>(
                                        this->heartbeat_topic_,
                                        qos, 
                                        std::bind(&PelicanUnit::storeHeartbeat, this, std::placeholders::_1),
                                        reentrant_opt
                                    );

    // TODO: needed by all agents?
    this->sub_to_request_vote_rpc_topic_ = this->create_subscription<comms::msg::RequestVoteRPC>(
                                        this->request_vote_rpc_topic_,
                                        qos, 
                                        std::bind(&PelicanUnit::expressPreference, this, std::placeholders::_1),
                                        reentrant_opt
                                    );

    std::chrono::milliseconds sleep_time {1000}; // TODO: value?

    this->hb_monitoring_timer_ = this->create_wall_timer(sleep_time, std::bind(&PelicanUnit::checkHeartbeat, this), this->reentrant_group_);
}

void PelicanUnit::becomeCandidate() {
    RCLCPP_INFO(get_logger(), "Becoming Candidate");
    this->role_ = candidate;
    this->leader_elected_ = false;

    this->election_completed_mutex_.lock();
    this->election_completed_ = false;
    this->election_completed_mutex_.unlock();

    this->sub_to_request_vote_rpc_topic_.reset(); // unsubscribe from topic

    this->election_timer_ = this->create_wall_timer(this->election_max_time_, std::bind(&PelicanUnit::setVotingTimedOut, this));

    // the candidate repeats this until:
    // (a) it wins the election
    // (b) another server establishes itself as leader
    // (c) a period of time goes by with no winner
    while(!this->checkElectionCompleted()) {
        this->current_term_++;
        while(!this->checkVotingCompleted() &&  !this->checkVotingTimedOut()) {
            this->vote(this->id_);
            this->requestVote();
        };
        // here either the voting is completed or it's timed-out
        this->leaderElection();
    };

}

bool PelicanUnit::isLeader() {
    RCLCPP_INFO(get_logger(), "Agent is leader? %d", this->role_ == leader);

    return (this->role_ == leader);
}
