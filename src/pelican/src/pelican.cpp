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

    this->sub_to_leader_selection_topic_ = this->create_subscription<comms::msg::Datapad>(
                                                this->leader_selection_topic_,
                                                qos, 
                                                std::bind(&PelicanUnit::leaderSelection, this, std::placeholders::_1)
                                            );

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

int PelicanUnit::get_ID() { return this->id_; };
std::string PelicanUnit::get_name() { return this->name_; };
std::string PelicanUnit::get_model() { return this->model_; };
double PelicanUnit::get_mass() {return this->mass_; };
possible_roles PelicanUnit::get_role() { return this->role_; };
int PelicanUnit::get_current_term() { return this->current_term_; };

void PelicanUnit::randomTimer() {
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

void PelicanUnit::leaderSelection(const comms::msg::Datapad::SharedPtr msg) const {
    std::cout << "\n\n";
    std::cout << "RECEIVED Datapad DATA"   << std::endl;
    std::cout << "============================="   << std::endl;
    std::cout << "term_id: " << msg->term_id << std::endl;
    std::cout << "voter_id: " << msg->voter_id << std::endl;
    std::cout << "proposed_leader: " << msg->proposed_leader << std::endl;

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
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = this->reentrant_group_;
    this->sub_to_heartbeat_topic_ = this->create_subscription<comms::msg::Heartbeat>(
                                        this->heartbeat_topic_,
                                        qos, 
                                        std::bind(&PelicanUnit::storeHeartbeat, this, std::placeholders::_1),
                                        sub_opt
                                    );

    std::chrono::milliseconds sleep_time {1000}; // TODO: value?

    this->hb_monitoring_timer_ = this->create_wall_timer(sleep_time, std::bind(&PelicanUnit::checkHeartbeat, this), this->reentrant_group_);
}

void PelicanUnit::becomeCandidate() {
    RCLCPP_INFO(get_logger(), "Becoming Candidate");
    this->role_ = candidate;

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
        this->becomeCandidate();
        this->hb_monitoring_timer_->cancel();
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