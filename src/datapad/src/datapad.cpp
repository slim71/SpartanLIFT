#include "datapad.hpp"

// Initialize the static instance pointer to a weak pointer
std::weak_ptr<Datapad> Datapad::instance_;

/************************** Ctors/Dctors ***************************/
Datapad::Datapad() : Node("Datapad"), logger_() {
    this->logger_.initSetup(std::make_shared<rclcpp::Logger>(this->get_logger()));
    this->running_mutex_.lock();
    this->running_ = true;
    this->running_mutex_.unlock();

    // Setting up the Reentrant group
    this->reentrant_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    this->reentrant_opt_.callback_group = this->reentrant_group_;

    // Own setup
    this->sub_to_channel = this->create_subscription<comms::msg::POI>(
        this->channel_topic_, this->data_qos_,
        std::bind(&Datapad::inspectAck, this, std::placeholders::_1), this->reentrant_opt_
    );
    this->pub_to_channel =
        this->create_publisher<comms::msg::POI>(this->channel_topic_, this->qos_value_);

    this->last_msg_ = std::make_unique<comms::msg::POI>();
    this->sendLogDebug(
        "last_msg_: {} {} {} {} {}", this->last_msg_->x, this->last_msg_->y, this->last_msg_->z,
        this->last_msg_->radius, this->last_msg_->ack
    );

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

void Datapad::inspectAck(const comms::msg::POI msg) {
    if (msg.ack) {
        this->sendLogInfo(
            "Received ack for position ({},{},{}) and radius {}", msg.x, msg.y, msg.z, msg.radius
        );
    } else {
        this->sendLogWarning(
            "Incorrect ack received with timestamp {}s.{}ns", msg.timestamp.sec,
            msg.timestamp.nanosec
        );
    }
}

void Datapad::sendPointOfInterest(float x, float y, float z, float radius) {
    this->sendLogInfo("Sending position ({},{},{}) and radius {}", x, y, z, radius);

    comms::msg::POI msg;
    msg.x = x;
    msg.y = y;
    msg.z = z;
    msg.radius = radius;
    msg.ack = false;
    msg.timestamp = this->now();

    this->pub_to_channel->publish(msg);
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

void Datapad::setInstance(rclcpp::Node::SharedPtr instance) {
    instance_ = std::static_pointer_cast<Datapad>(instance);
}

std::shared_ptr<Datapad> Datapad::getInstance() {
    return instance_.lock();
}
