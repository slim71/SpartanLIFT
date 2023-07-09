#include "pelican.hpp"
#include "pugixml.hpp"

// TODO: specialize logs with agent ID

PelicanUnit::PelicanUnit() : Node("single_pelican") {
    // The subscription sets a QoS profile based on rmw_qos_profile_sensor_data. 
    // This is needed because the default ROS 2 QoS profile for subscribers is 
    // incompatible with the PX4 profile for publishers.
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    
    this->exclusive_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->reentrant_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Subscriber, listening for VehicleLocalPosition messages
    // In NED. The coordinate system origin is the vehicle position at the time when the EKF2-module was started.
    this->sub_to_local_pos_topic_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                                        this->local_pos_topic_,
                                        qos, 
                                        std::bind(&PelicanUnit::printData, this, std::placeholders::_1)
                                    );

    // TODO: add group
    this->sub_to_leader_selection_topic_ = this->create_subscription<comms::msg::Datapad>(
                                                this->leader_selection_topic_,
                                                qos, 
                                                std::bind(&PelicanUnit::storeCandidacy, this, std::placeholders::_1)
                                            );
    // TODO: add group
    this->pub_to_leader_selection_topic_ = this->create_publisher<comms::msg::Datapad>(
                                                this->leader_selection_topic_, 10
                                            ); // TODO: 10 (or better value) in constant

    // Declare parameters
    declare_parameter("name", ""); // default to ""
    declare_parameter("model", ""); // default to ""

    // Get parameters values and store them
    get_parameter("name", name_);
    get_parameter("model", model_);

    parseModel();

    // Log parameters values
    RCLCPP_INFO(get_logger(), "Copter %s loaded model %s", name_.c_str(), model_.c_str());

    this->becomeFollower();
}

PelicanUnit::~PelicanUnit() {
    RCLCPP_INFO(get_logger(), "Destructor for %s", name_.c_str());

    // Cancel all timers; no problem arises if they're not initialized
    this->hb_transmission_timer_->cancel();
    this->hb_monitoring_timer_->cancel();
    this->timer_->cancel();
}

// TODO: use these when needed
int PelicanUnit::get_ID() { return this->id_; };
std::string PelicanUnit::get_name() { return this->name_; };
std::string PelicanUnit::get_model() { return this->model_; };
double PelicanUnit::get_mass() {return this->mass_; };
possible_roles PelicanUnit::get_role() { return this->role_; };
int PelicanUnit::get_current_term() { return this->current_term_; };

void PelicanUnit::randomTimer() { // TODO: modify and use
    RCLCPP_INFO(get_logger(), "Creating timer");

    // initialize random seed
    srand(time(NULL));
    
    // TODO: do I need a particular kind of random?
    this->timer_ = this->create_wall_timer(std::chrono::seconds(rand()%10+1), std::bind(&PelicanUnit::isLeader, this)); // TODO: change callback and duration
}

void PelicanUnit::parseModel() {
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(this->model_.c_str());

    if (result) {
        pugi::xml_node start = doc.child("sdf").child("model");

        for (pugi::xml_node link = start.first_child(); link; link = link.next_sibling()) {
            if (strcmp(link.attribute("name").value(), "base_link") == 0) {
                this->mass_ = link.child("inertial").child("mass").text().as_double();
                RCLCPP_INFO(get_logger(), "Drone mass: %f", this->mass_);
            }
        }
    } else {
        RCLCPP_ERROR(get_logger(), "Model file could not be loaded! Error description: %s", result.description());
        // TODO: abort everything
    }
}

void PelicanUnit::storeCandidacy(const comms::msg::Datapad::SharedPtr msg) {
    std::cout << "\n\n";
    std::cout << "AGENT " << this->id_ << "RECEIVED DATAPAD DATA"   << std::endl;
    std::cout << "============================="   << std::endl;
    std::cout << "term_id: " << msg->term_id << std::endl;
    std::cout << "voter_id: " << msg->voter_id << std::endl;
    std::cout << "proposed_leader: " << msg->proposed_leader << std::endl;
    std::cout << "candidate_mass: " << msg->candidate_mass << std::endl;

    this->votes_mutex_.lock();
    this->received_votes.push_back(msg);
    this->votes_mutex_.unlock();

    // Votes received are supposedly from followers, so if no more votes come within a time frame, the elections is finished
    if(this->role_ == candidate) {
        if(this->voting_timer != nullptr) { // Make the timer restart
            this->voting_timer->reset();
        } else{
            this->voting_timer = this->create_wall_timer(this->voting_max_time_, std::bind(&PelicanUnit::setElectionCompleted, this));
        };
    };

}

void PelicanUnit::setElectionCompleted() {
    this->voting_timer->cancel();
    std::lock_guard<std::mutex> lock(this->election_completed_mutex_);
    this->election_completed_ = true;
}

void PelicanUnit::flushVotes() {
    // Ensure safe access to received_votes
    std::lock_guard<std::mutex> lock(this->votes_mutex_);
    this->received_votes.clear();
}

void PelicanUnit::printData(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) const {
    std::cout << "\n\n";
    std::cout << "RECEIVED VehicleLocalPosition DATA"   << std::endl;
    std::cout << "============================="   << std::endl;
    std::cout << "Time since start: " << msg->timestamp << std::endl; // [us]
    std::cout << "Timestamp of raw data: " << msg->timestamp_sample << std::endl; // TODO: not needed? // [us]
    std::cout << "x: " << msg->x << std::endl; // [m]
    std::cout << "y: " << msg->y << std::endl; // [m]
    std::cout << "z: " << msg->z << std::endl; // [m]
    std::cout << "vx: " << msg->vx << std::endl; // [m/s]
    std::cout << "vy: " << msg->vy << std::endl; // [m/s]
    std::cout << "vz: " << msg->vz << std::endl; // [m/s]
    std::cout << "xy_valid: " << msg->xy_valid << std::endl; // true if x and y are valid
    std::cout << "z_valid: " << msg->z_valid << std::endl; // true if z is valid
    std::cout << "v_xy_valid: " << msg->xy_valid << std::endl; // true if vx and vy are valid
    std::cout << "v_z_valid: " << msg->z_valid << std::endl; // true if vz is valid
    std::cout << "ax: " << msg->ax << std::endl; // [m/s^2]
    std::cout << "ay: " << msg->ay << std::endl; // [m/s^2]
    std::cout << "az: " << msg->az << std::endl; // [m/s^2]
    std::cout << "heading: " << msg->heading << std::endl; // [rad]
}

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

void PelicanUnit::leaderElection() {
    // As for followers, even the candidates has stored all the votes
    // TODO: case of no votes inserted
    
    // Copy because I don't want to modify the original vector TODO: needed?
    std::vector<comms::msg::Datapad::SharedPtr> votes_copy(received_votes);
    std::sort(votes_copy.begin(), 
              votes_copy.end(), 
              [](const comms::msg::Datapad::SharedPtr &a, const comms::msg::Datapad::SharedPtr &b) {
                  return a->proposed_leader < b->proposed_leader;
              }
            );

    std::vector<vote_count> ballot;

    for(auto it = std::cbegin(votes_copy); it != std::cend(votes_copy); ) {  
        vote_count candidate_support;
        candidate_support.candidate_id = (*it)->proposed_leader;
        candidate_support.total = std::count_if(it, 
                                                std::cend(votes_copy), 
                                                [&](const comms::msg::Datapad::SharedPtr &v) {
                                                    return v->proposed_leader == (*it)->proposed_leader;
                                                }
                                                );
        ballot.push_back(candidate_support);

        // Increment the iterator until the last value of the cluster is found
        for(auto last = (*it)->proposed_leader; (*++it)->proposed_leader == last;);
    }

    std::sort(ballot.begin(),
              ballot.end(),
              [](vote_count a, vote_count b){
                  return a.total < b.total;
              }
            );
    auto item = std::find_if(ballot.begin(),
                             ballot.end(),
                             [this](const vote_count &v){
                                 return v.candidate_id == this->get_ID();
                             }
                            );
    
    // If item is the last element of the ballot vector, either this candidate has won the election or there's a tie
    if((*item).total == (*ballot.cend()).total) {
        if((*item).total != (*(ballot.cend()-1)).total) { // this candidate has won
            // I've been chosen!
            this->election_timer_->cancel();
            this->election_completed_ = true;
            this->leader_elected_ = true;
            this->leader_id_ = this->id_;
            this->becomeLeader();
            this->sendHeartbeat(); // to be sure one is sent now
        } else { // tie
            // each candidate will time out and start a new election by incrementing its term and initiating another round of Request-Vote RPCs
            // Do not flag the election as completed and starts from scratch
            this->resetVotingWindow();
        }
    };
    // Otherwise, let the winning candidate send its heartbeat as leader confirmation
}

bool PelicanUnit::checkVotingTimedOut() {
    // Ensure safe access to election_timed_out
    std::lock_guard<std::mutex> lock(this->election_timedout_mutex_);
    return this->election_timed_out;
}

bool PelicanUnit::checkElectionCompleted() {
    // Ensure safe access to election_timed_out
    std::lock_guard<std::mutex> lock(this->election_completed_mutex_);
    return this->election_completed_;
}

bool PelicanUnit::checkVotingCompleted() {
    // Ensure safe access to election_timed_out
    std::lock_guard<std::mutex> lock(this->voting_completed_mutex_);
    return this->voting_completed_;
}

void PelicanUnit::setVotingTimedOut() {
    this->election_timer_->cancel();
    // Ensure safe access to election_timed_out
    std::lock_guard<std::mutex> lock(this->election_timedout_mutex_);
    this->election_timed_out = true;
}

void PelicanUnit::setVotingCompleted() {
    this->election_timer_->cancel();
    // Ensure safe access to election_timed_out
    std::lock_guard<std::mutex> lock(this->election_timedout_mutex_);
    this->election_timed_out = true;
}

void PelicanUnit::resetVotingWindow() {
    this->voting_completed_mutex_.lock();
    this->voting_completed_ = false;
    this->voting_completed_mutex_.unlock();

    this->election_timer_->reset();
    this->election_timedout_mutex_.lock();
    this->election_timed_out = false;
    this->election_timedout_mutex_.unlock();
}

bool PelicanUnit::isLeader() {
    RCLCPP_INFO(get_logger(), "Agent is leader? %d", this->role_ == leader);

    return (this->role_ == leader);
}

void PelicanUnit::vote(int id_to_vote) {
    RCLCPP_INFO(get_logger(), "Voting: %d", id_to_vote);

    auto msg = comms::msg::Datapad();

    // int64 term_id
    // int64 voter_id
    // int64 proposed_leader
    msg.term_id = this->current_term_;
    msg.voter_id = this->id_;
    msg.proposed_leader = id_to_vote;

    this->pub_to_leader_selection_topic_->publish(msg);
}

void PelicanUnit::requestVote() {
    if (this->pub_to_request_vote_rpc_topic_ == nullptr) {
        this->pub_to_request_vote_rpc_topic_ = this->create_publisher<comms::msg::RequestVoteRPC>(
                                                    this->request_vote_rpc_topic_, 10
                                                ); // TODO: 10 (or better value) in constant
    };

    comms::msg::RequestVoteRPC req;
    req.do_vote = true;
    req.solicitant_id = this->id_;
    req.term_id = this->current_term_;

    this->pub_to_request_vote_rpc_topic_->publish(req);
}

void PelicanUnit::expressPreference(const comms::msg::RequestVoteRPC msg) { // TODO: think about the name
    this->sub_to_request_vote_rpc_topic_.reset();
    auto heavier = std::max_element(this->received_votes.begin(),
                                    this->received_votes.end(),
                                    [](comms::msg::Datapad::SharedPtr first, comms::msg::Datapad::SharedPtr second) {
                                        return first->candidate_mass > second->candidate_mass;
                                    }
                                    );
    this->vote((*heavier)->candidate_mass);
}

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