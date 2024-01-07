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

void Datapad::landingPage() {
    std::cout << "Select functionality" << std::endl;
    int choice;
    while (true) {
        std::cout << "(1) Ensure leader is elected" << std::endl;
        std::cout << "(2) Initiate takeoff" << std::endl;
        std::cout << "(3) Send payload position" << std::endl;
        std::cout << "(4) Send dropoff position" << std::endl;
        std::cout << "(5) Initiate landing" << std::endl;
        std::cout << "(0) Exit" << std::endl;
        std::cout << " >>> ";

        std::cin >> choice;

        this->running_mutex_.lock();
        auto ret = this->running_;
        this->running_mutex_.unlock();
        if (!ret)
            break;

        if (std::cin.fail()) {
            this->sendLogDebug("Wrong input!");
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Input is not a number!" << std::endl << std::endl;
            // break;
        } else {
            this->sendLogDebug("Choice made: {}", choice);
            switch (choice) {
                case 0:
                    break;
                case 1:
                    this->contactLeader();
                    break;
                case 2:
                    break;
                case 3:
                    break;
                case 4:
                    break;
                case 5:
                    break;
                default:
                    this->sendLogWarning("Choice not supported!");
            }
        }

        // Simulate some wait between loop iterations
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    };

    std::cout << "Ending..." << std::endl;
    this->sendLogDebug("Ending main functionality...");
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

void Datapad::contactLeader() {
    rclcpp::Client<comms::srv::Contact>::SharedPtr client =
        this->create_client<comms::srv::Contact>("contactLeader_client");

    auto request = std::make_shared<comms::srv::Contact::Request>();
    request->question = true;

    // Search for a second, then log and search again
    int total_search_time = 0;
    while (!client->wait_for_service(std::chrono::seconds(SEARCH_LEADER_STEP)) &&
           total_search_time < MAX_SEARCH_TIME) {
        if (!rclcpp::ok()) {
            this->sendLogError("Client interrupted while waiting for service. Terminating...");
            return; // CHECK: do something else?
        }

        this->sendLogDebug("Service not available; waiting some more...");
        total_search_time += SEARCH_LEADER_STEP;
    };

    if (total_search_time < MAX_SEARCH_TIME) {
        this->sendLogDebug("Server available");
        auto result = client->async_send_request(
            request, std::bind(&Datapad::processContact, this, std::placeholders::_1)
        );
    } else {
        this->sendLogWarning("The server seems to be down. Please try again");
    }

    // Wait for a response
}

void Datapad::processContact(rclcpp::Client<comms::srv::Contact>::SharedFuture future) {
    // Wait for 1s or untile the result is available
    auto status = future.wait_for(std::chrono::seconds(1));

    if (status == std::future_status::ready) {
        auto response = future.get();
        if (response->present) {
            this->sendLogInfo("Agent {} is currently the fleet leader", response->leader_id);
            this->leader_present_ = true;
        } else {
            this->sendLogInfo("The fleet has no leader yet");
            this->leader_present_ = false; // CHECK: redundant?
        }
    } else {
        this->sendLogDebug("Service In-Progress...");
    }
}
