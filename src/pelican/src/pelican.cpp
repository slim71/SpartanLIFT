#include "pelican.hpp"
#include "pugixml.hpp"

PelicanUnit::PelicanUnit() : Node("single_pelican") {
    // The subscription sets a QoS profile based on rmw_qos_profile_sensor_data. 
    // This is needed because the default ROS 2 QoS profile for subscribers is 
    // incompatible with the PX4 profile for publishers.
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    
    // Subscriber, listening for VehicleLocalPosition messages
    // In NED. The coordinate system origin is the vehicle position at the time when the EKF2-module was started.
    this->sub_to_local_pos_topic_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                                        this->local_pos_topic_,
                                        qos, 
                                        std::bind(&PelicanUnit::printData, this, std::placeholders::_1)
                                    );

    this->sub_to_leader_selection_topic_ = this->create_subscription<comms::msg::Datapad>(
                                                this->local_pos_topic_,
                                                qos, 
                                                std::bind(&PelicanUnit::leaderSelection, this, std::placeholders::_1)
                                            );

    this->pub_to_leader_selection_topic_ = this->create_publisher<comms::msg::Datapad>(
                                                this->leader_selection_topic_, 10
                                            ); // TODO: 10 in constant

    // Declare parameters
    declare_parameter("name", ""); // default to ""
    declare_parameter("model", ""); // default to ""

    // Get parameters values and store them
    get_parameter("name", name_);
    get_parameter("model", model_);

    parseModel();

    // Log parameters values
    RCLCPP_INFO(get_logger(), "Copter %s loaded model %s", name_.c_str(), model_.c_str());

    this->becomeCandidate();
}

PelicanUnit::~PelicanUnit() {
    RCLCPP_INFO(get_logger(), "Destructor for %s", name_.c_str());
}

int PelicanUnit::get_ID() { return this->id_; };
std::string PelicanUnit::get_name() { return this->name_; };
std::string PelicanUnit::get_model() { return this->model_; };
double PelicanUnit::get_mass() {return this->mass_; };
possible_roles PelicanUnit::get_role() { return this->role_; };

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
    this->role_ = leader;
}

void PelicanUnit::becomeFollower() {
    this->role_ = follower;
}

void PelicanUnit::becomeCandidate() {
    this->role_ = candidate;

}

void PelicanUnit::vote(int id_to_vote) {
    auto msg = comms::msg::Datapad();

    // int64 term_id
    // int64 voter_id
    // int64 proposed_leader
    msg.term_id = this->current_term_;
    msg.voter_id = this->id_;
    msg.proposed_leader = id_to_vote;

    this->pub_to_leader_selection_topic_->publish(msg);
}