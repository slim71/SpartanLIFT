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
    sub_to_local_pos_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                                this->local_pos_topic_,
                                qos, 
                                std::bind(&PelicanUnit::printData, this, std::placeholders::_1));

    sub_to_leader_topic_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                                this->local_pos_topic_,
                                qos, 
                                std::bind(&PelicanUnit::printData, this, std::placeholders::_1)); // TODO: define callback

    // Declare parameters
    declare_parameter("name", ""); // default to ""
    declare_parameter("model", ""); // default to ""

    // Get parameters values and store them
    get_parameter("name", name_);
    get_parameter("model", model_);

    parseModel();

    // Log parameters values
    RCLCPP_INFO(get_logger(), "Copter %s loaded model %s", name_.c_str(), model_.c_str());
}

PelicanUnit::~PelicanUnit() {
    RCLCPP_INFO(get_logger(), "Destructor for %s", name_.c_str());
}

int PelicanUnit::get_ID() {};
std::string PelicanUnit::get_name() {};
std::string PelicanUnit::get_model() {};
double PelicanUnit::get_mass() {};
possible_roles PelicanUnit::get_role() {};

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

void PelicanUnit::leaderSelection(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) const {

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
