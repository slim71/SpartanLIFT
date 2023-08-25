#include "pelican.hpp"
#include "pugixml.hpp"

// Initialize the static instance pointer to a weak pointer
std::weak_ptr<Pelican> Pelican::instance_;

Pelican::Pelican() : Node("Pelican") {
    // Declare parameters
    declare_parameter("model", ""); // default to ""
    declare_parameter("id", 0);     // default to 0

    // Get parameters values and store them
    get_parameter("model", this->model_);
    get_parameter("id", this->id_);

    this->reentrant_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    // Callbacks belonging to different callback groups (of any type) can always be executed
    // parallel to each other Almost every callback can be executed in parallel to one another or
    // with itself (thanks to mutexes)
    this->reentrant_opt_.callback_group = this->reentrant_group_;

    // Subscriber, listening for VehicleLocalPosition messages
    // In NED. The coordinate system origin is the vehicle position at the time when the EKF2-module
    // was started. Needed by every type of agent and never canceled
    this->sub_to_local_pos_topic_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        this->local_pos_topic_, this->px4_qos_,
        std::bind(&Pelican::printData, this, std::placeholders::_1), this->reentrant_opt_
    );

    this->parseModel();

    // Log parameters values
    this->logInfo("Loaded model {} | Agent mass: {}", this->getModel(), this->getMass());

    this->becomeFollower();
}

Pelican::~Pelican() {
    // TODO: Check for resource leaks: Make sure that you are releasing all resources (memory,
    //  publishers, subscribers, timers, etc.) correctly when shutting down the node.
    //  If there are any resource leaks, it can lead to invalid contexts or other errors.

    this->logDebug("Destructor for agent {}", this->getID());

    // Cancel all timers; no problem arises if they're not initialized
    this->hb_transmission_timer_->cancel();

    this->logDebug("Trying to kill the thread");
    this->stopBallotThread();
}

void Pelican::parseModel() {
    this->logDebug("Trying to load model {}", this->getModel());
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(this->getModel().c_str());

    if (result) {
        pugi::xml_node start = doc.child("sdf").child("model");

        for (pugi::xml_node link = start.first_child(); link; link = link.next_sibling()) {
            if (strcmp(link.attribute("name").value(), "base_link") == 0) {
                this->setMass(link.child("inertial").child("mass").text().as_double());
            }
        }
    } else {
        this->logError(
            "Model file {} could not be loaded! Error description: {}", this->getModel(),
            result.description()
        );
        // Abort everything
        throw std::runtime_error("Agent model could not be parsed!");
    }
}

void Pelican::printData(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) const {
    std::cout << "\n\n";
    std::cout << "RECEIVED VehicleLocalPosition DATA" << std::endl;
    std::cout << "=============================" << std::endl;
    std::cout << "Time since start: " << msg->timestamp << std::endl;             // [us]
    std::cout << "Timestamp of raw data: " << msg->timestamp_sample << std::endl; // [us]
    std::cout << "x: " << msg->x << std::endl;                                    // [m]
    std::cout << "y: " << msg->y << std::endl;                                    // [m]
    std::cout << "z: " << msg->z << std::endl;                                    // [m]
    std::cout << "vx: " << msg->vx << std::endl;                                  // [m/s]
    std::cout << "vy: " << msg->vy << std::endl;                                  // [m/s]
    std::cout << "vz: " << msg->vz << std::endl;                                  // [m/s]
    std::cout << "xy_valid: " << msg->xy_valid << std::endl;   // true if x and y are valid
    std::cout << "z_valid: " << msg->z_valid << std::endl;     // true if z is valid
    std::cout << "v_xy_valid: " << msg->xy_valid << std::endl; // true if vx and vy are valid
    std::cout << "v_z_valid: " << msg->z_valid << std::endl;   // true if vz is valid
    std::cout << "ax: " << msg->ax << std::endl;               // [m/s^2]
    std::cout << "ay: " << msg->ay << std::endl;               // [m/s^2]
    std::cout << "az: " << msg->az << std::endl;               // [m/s^2]
    std::cout << "heading: " << msg->heading << std::endl;     // [rad]
}

void Pelican::signalHandler(int signum) {
    // Stop the thread gracefully
    std::shared_ptr<Pelican> node = getInstance();
    if (node) {
        node->stopBallotThread();
    }

    rclcpp::shutdown();
}

void Pelican::startBallotThread() {
    if (!this->ballot_thread_.joinable()) {
        this->ballot_thread_ = std::thread(&Pelican::ballotCheckingThread, this);
    }
}

void Pelican::stopBallotThread() {
    if (this->ballot_thread_.joinable()) {
        this->setIsTerminated();
        this->ballot_thread_.join();
    }
}

void Pelican::prepareCommonCallbacks() { // for followers and candidates
    /******************* Subscribrers ****************************/
    if (!this->sub_to_leader_election_topic_) {
        this->sub_to_leader_election_topic_ = this->create_subscription<comms::msg::Datapad>(
            this->leader_election_topic_, this->standard_qos_,
            std::bind(&Pelican::storeCandidacy, this, std::placeholders::_1),
            this->reentrant_opt_
        );
    }

    if (!this->pub_to_leader_election_topic_) {
        this->pub_to_leader_election_topic_ = this->create_publisher<comms::msg::Datapad>(
            this->leader_election_topic_, this->standard_qos_
        );
    }

    // all agents, to avoid multiple leaders (this should not happen, since Raft guarantees safety)
    if (!this->sub_to_heartbeat_topic_) {
        this->sub_to_heartbeat_topic_ = this->create_subscription<comms::msg::Heartbeat>(
            this->heartbeat_topic_, this->standard_qos_,
            std::bind(&Pelican::storeHeartbeat, this, std::placeholders::_1),
            this->reentrant_opt_
        );
    }

    /******************* Publishers ****************************/
    this->resetSharedPointer(this->pub_to_heartbeat_topic_);

    /*********************** Timers ****************************/
    this->cancelTimer(this->hb_transmission_timer_);
}
