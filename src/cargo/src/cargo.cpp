#include "cargo.hpp"

// Initialize the static instance pointer to a weak pointer
std::weak_ptr<Cargo> Cargo::instance_;

/************************** Ctors/Dctors ***************************/
Cargo::Cargo() : Node("Cargo"), logger_() {
    this->logger_.initSetup(std::make_shared<rclcpp::Logger>(this->get_logger()));
    this->setRunning();

    // Callback groups
    this->exc_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->exc_group_opt_.callback_group = this->exc_group_;

    // Data exchange
    this->sub_to_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->odom_topic_, this->data_qos_,
        std::bind(&Cargo::storeCargoOdometry, this, std::placeholders::_1), this->exc_group_opt_
    );
    this->cargopoint_server_ = this->create_service<comms::srv::CargoPoint>(
        "cargopoint_service",
        std::bind(&Cargo::shareCargoPosition, this, std::placeholders::_1, std::placeholders::_2)
    );
}

Cargo::~Cargo() {
    this->sendLogDebug("Destructor for Cargo node");
}

void Cargo::signalHandler(int signum) {
    // Only handle SIGINT
    std::cout << "Some signal received..." << std::endl;

    if (signum == SIGINT) {
        // Stop the thread gracefully
        std::cout << "SIGINT received!" << std::endl;
        std::shared_ptr<Cargo> node = getInstance();
        if (node) {
            node->unsetRunning();
        } else
            std::cout << "No node instance to consider" << std::endl;

        rclcpp::shutdown();
    }
}

std::shared_ptr<Cargo> Cargo::getInstance() {
    return instance_.lock();
}

void Cargo::setInstance(rclcpp::Node::SharedPtr instance) {
    instance_ = std::static_pointer_cast<Cargo>(instance);
}

bool Cargo::isRunning() const {
    std::lock_guard lock(this->running_mutex_);
    return this->running_;
}

void Cargo::storeCargoOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    this->sendLogDebug("Storing {}", msg->pose.pose.position);
    this->odom_mutex_.lock();
    this->own_odom_ = msg->pose.pose.position;
    this->odom_mutex_.unlock();
}

void Cargo::setRunning() {
    std::lock_guard lock(this->running_mutex_);
    this->running_ = true;
}

void Cargo::unsetRunning() {
    std::lock_guard lock(this->running_mutex_);
    this->running_ = false;
}

void Cargo::shareCargoPosition(
    const std::shared_ptr<comms::srv::CargoPoint::Request>,
    const std::shared_ptr<comms::srv::CargoPoint::Response> response
) {
    std::lock_guard lock(this->odom_mutex_);
    response->position = this->own_odom_;
}
