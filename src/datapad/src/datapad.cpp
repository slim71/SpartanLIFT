#include "datapad.hpp"

// Initialize the static instance pointer to a weak pointer
std::weak_ptr<Datapad> Datapad::instance_;

/************************** Ctors/Dctors ***************************/
Datapad::Datapad() : Node("Datapad"), logger_() {
    this->logger_.initSetup(std::make_shared<rclcpp::Logger>(this->get_logger()));
    this->running_mutex_.lock();
    this->running_ = true;
    this->running_mutex_.unlock();

    // Callback groups
    this->reentrant_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    this->reentrant_opt_.callback_group = this->reentrant_group_;

    // Own setup
    this->teleopdata_client_ =
        rclcpp_action::create_client<comms::action::TeleopData>(this, "contactLeader");
    this->cargopoint_client_ = this->create_client<comms::srv::CargoPoint>("cargopoint_service");

    this->setup_timer_ = this->create_wall_timer(
        this->setup_timeout_,
        [this]() {
            this->setup_timer_->cancel();
            this->sendLogDebug("Initiating main functionality");
            this->landingPage();
        },
        this->reentrant_group_
    );
}

Datapad::~Datapad() {
    this->sendLogDebug("Destructor for Datapad node");
}

void Datapad::signalHandler(int signum) {
    // Only handle SIGINT
    std::cout << "Some signal received..." << std::endl;

    if (signum == SIGINT) {
        // Stop the thread gracefully
        std::cout << "SIGINT received!" << std::endl;
        std::shared_ptr<Datapad> node = getInstance();
        if (node) {
            node->running_mutex_.lock();
            node->running_ = false;
            node->running_mutex_.unlock();
            // Another input or CTRL-C will be needed to completely close
            // all computations, due to cin's blocking behavior
        } else
            std::cout << "No node instance to consider" << std::endl;

        rclcpp::shutdown();
    }
}

void Datapad::analyzeTeleopDataResponse(
    const rclcpp_action::ClientGoalHandle<comms::action::TeleopData>::SharedPtr& goal_handle
) {
    if (!goal_handle) {
        this->sendLogWarning("Goal was rejected by the server!");
    } else {
        this->sendLogDebug("Goal accepted by server, waiting for result...");
    }
}

void Datapad::parseTeleopDataFeedback(
    rclcpp_action::ClientGoalHandle<comms::action::TeleopData>::SharedPtr,
    const std::shared_ptr<const comms::action::TeleopData::Feedback> feedback
) {
    this->sendLogDebug(
        "Feedback received: agents_involved: {}, last_joined: {}, command: {}, execution: {}",
        feedback->agents_involved, feedback->last_joined, feedback->command, feedback->execution
    );
}

void Datapad::storeCargoPoint(rclcpp::Client<comms::srv::CargoPoint>::SharedFuture future) {
    // Wait for the specified amount or until the result is available
    this->sendLogDebug("Getting response...");
    auto status = future.wait_for(std::chrono::seconds(constants::SERVICE_FUTURE_WAIT_SECS));

    if (status == std::future_status::ready) {
        auto response = future.get();
        this->sendLogInfo("Received position: {}", response->position);
        this->cargo_odom_ = response->position;
        this->setAndNotifyCargoHandled();
    } else {
        this->sendLogDebug("Service not ready yet...");
        this->unsetAndNotifyCargoHandled();
    }
}
