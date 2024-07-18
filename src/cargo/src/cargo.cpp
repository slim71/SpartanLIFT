#include "cargo.hpp"

// Initialize the static instance pointer to a weak pointer
std::weak_ptr<Cargo> Cargo::instance_;

/************************** Ctors/Dctors ***************************/
Cargo::Cargo() : Node("Cargo"), logger_() {
    // Declare parameters
    declare_parameter("model", "cargo"); // default to "cargo"
    declare_parameter("world", "empty"); // default to "empty"

    // Get parameters values and store them
    get_parameter("model", this->model_name_);
    get_parameter("world", this->world_name_);

    this->logger_.initSetup(std::make_shared<rclcpp::Logger>(this->get_logger()));
    this->setRunning();

    // Log parameters
    this->sendLogInfo("Cargo spawned. Model: {}, world: {}", this->model_name_, this->world_name_);

    // Callback groups
    this->own_odom_exc_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->own_odom_exc_group_opt_.callback_group = this->own_odom_exc_group_;
    this->leader_odom_exc_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->leader_odom_exc_group_opt_.callback_group = this->leader_odom_exc_group_;
    this->pose_exc_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->pose_exc_group_opt_.callback_group = this->pose_exc_group_;

    // Topics
    this->odom_topic_ = "/model/" + this->model_name_ + "/odometry";
    this->set_pose_service_ = "/world/" + this->world_name_ + "/set_pose";

    // Data exchange
    this->sub_to_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->odom_topic_, this->data_qos_,
        std::bind(&Cargo::storeCargoOdometry, this, std::placeholders::_1),
        this->own_odom_exc_group_opt_
    );
    this->cargopoint_server_ = this->create_service<comms::srv::CargoPoint>(
        "cargopoint_service",
        std::bind(&Cargo::shareCargoPosition, this, std::placeholders::_1, std::placeholders::_2)
    );
    this->attachment_server_ = this->create_service<comms::srv::CargoLinkage>(
        "attachment_service",
        std::bind(&Cargo::notifyAttachment, this, std::placeholders::_1, std::placeholders::_2)
    );
    this->set_pose_client_ =
        this->create_client<ros_gz_interfaces::srv::SetEntityPose>(this->set_pose_service_);
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

void Cargo::shareCargoPosition(
    const std::shared_ptr<comms::srv::CargoPoint::Request>,
    const std::shared_ptr<comms::srv::CargoPoint::Response> response
) {
    std::lock_guard lock(this->odom_mutex_);
    response->position = this->own_odom_;
}

// CargoLinkage request handler
void Cargo::notifyAttachment(
    const std::shared_ptr<comms::srv::CargoLinkage::Request> req,
    std::shared_ptr<comms::srv::CargoLinkage::Response> response
) {
    if (!req) {
        this->sendLogError("Request is empty");
        response->done = false;
        return;
    }
    if (req->leader_id <= 0) {
        this->sendLogError("ID in request is invalid");
        response->done = false;
        return;
    }
    if (req->model.empty()) {
        this->sendLogError("Model in request is null");
        response->done = false;
        return;
    }

    this->leader_odom_topic_ =
        "/model/" + req->model + "_" + std::to_string(req->leader_id) + "/odometry";
    this->sub_to_leader_odometry_topic_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->leader_odom_topic_, this->data_qos_,
        std::bind(&Cargo::recordLeaderPosition, this, std::placeholders::_1),
        this->leader_odom_exc_group_opt_
    );

    this->startFollowing();
    response->done = true;
}

void Cargo::recordLeaderPosition(const nav_msgs::msg::Odometry::SharedPtr msg) {
    this->sendLogDebug("Leader is at {}", msg->pose.pose.position);
    std::lock_guard lock(this->reference_mutex_);
    this->reference_buffer_.push_back(*msg);
}

void Cargo::startFollowing() {
    this->sendLogDebug("Starting to follow the leader position...");
    this->following_timer_ = this->create_wall_timer(
        this->following_timeout_, std::bind(&Cargo::followReference, this), this->pose_exc_group_
    );
}

void Cargo::stopFollowing() {
    this->sendLogDebug("Stopping leader following...");
    if (this->following_timer_)
        this->following_timer_->cancel();
}

void Cargo::followReference() {
    // Get reference data
    nav_msgs::msg::Odometry to_follow;
    this->sendLogDebug("getting first element");
    this->reference_mutex_.lock();
    if (!this->reference_buffer_.empty()) {
        to_follow = this->reference_buffer_[0];
    } else {
        this->sendLogDebug("No leader position yet");
        this->reference_mutex_.unlock();
        return;
    }
    this->reference_buffer_.pop_front();
    this->reference_mutex_.unlock();
    this->sendLogDebug("got and removed {}", to_follow);

    // Build request
    ros_gz_interfaces::srv::SetEntityPose::Request req;
    req.entity.set__name(this->model_name_);
    req.pose.position.set__x(to_follow.pose.pose.position.x);
    req.pose.position.set__y(to_follow.pose.pose.position.y);
    req.pose.position.set__z(
        (to_follow.pose.pose.position.z > 2) ? to_follow.pose.pose.position.z - 2 : 0
    );
    this->sendLogDebug(
        "Sumup: {}-{} ({},{},{})", req.entity.id, req.entity.name, req.pose.position.x,
        req.pose.position.y, req.pose.position.z
    );

    // Send request
    // Search for a second, then log and search again if needed
    unsigned int total_search_time = 0;
    while (!this->set_pose_client_->wait_for_service(
               std::chrono::seconds(constants::SEARCH_SERVER_STEP_SECS)
           ) &&
           total_search_time < constants::MAX_SEARCH_TIME_SECS) {
        if (!rclcpp::ok()) {
            this->sendLogError("Client interrupted while waiting for service. Terminating...");
            return;
        }

        this->sendLogDebug("Service not available; waiting some more...");
        total_search_time += constants::SEARCH_SERVER_STEP_SECS;
    };

    if (total_search_time < constants::MAX_SEARCH_TIME_SECS) {
        this->sendLogDebug("SetPoseEntity server available");

        auto request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>(req);
        // Send request
        auto async_request_result = this->set_pose_client_->async_send_request(
            request, std::bind(&Cargo::parseSetEntityPoseResponse, this, std::placeholders::_1)
        );

        auto future_status =
            async_request_result.wait_for(std::chrono::seconds(constants::SERVICE_FUTURE_WAIT_SECS)
            );
        if (!async_request_result.valid() || (future_status != std::future_status::ready)) {
            this->sendLogWarning("Failed to receive confirmation from the SetEntityPose server!");
            this->set_pose_client_->prune_pending_requests();
            return;
        }
    } else {
        this->sendLogWarning("The server seems to be down. Please try again.");
        return;
    }
}

void Cargo::parseSetEntityPoseResponse(
    rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedFuture future
) {
    // Wait for the specified amount or until the result is available
    this->sendLogDebug("Getting response...");
    auto status = future.wait_for(std::chrono::seconds(constants::SERVICE_FUTURE_WAIT_SECS));

    if (status == std::future_status::ready) {
        auto response = future.get();
        if (response->success)
            this->sendLogInfo("Pose changed successfully");
        else
            this->sendLogWarning("Error while setting cargo pose");
    } else {
        this->sendLogDebug("Service not ready yet...");
    }
}
