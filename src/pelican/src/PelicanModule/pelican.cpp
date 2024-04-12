#include "PelicanModule/pelican.hpp"

// Initialize the static instance pointer to a weak pointer
std::weak_ptr<Pelican> Pelican::instance_;

/************************** Ctors/Dctors ***************************/
Pelican::Pelican()
    : Node("Pelican"),
      logger_(),
      hb_core_(this),
      el_core_(this),
      tac_core_(this),
      unsc_core_(this) {
    // Declare parameters
    declare_parameter("model", ""); // default to ""
    declare_parameter("id", 0);     // default to 0
    declare_parameter("roi", 1.0);  // default to 1

    // Get parameters values and store them
    get_parameter("model", this->model_);
    get_parameter("id", this->id_);
    get_parameter("roi", this->roi_);

    // Setting up the Reentrant group
    this->reentrant_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    this->reentrant_opt_.callback_group = this->reentrant_group_;

    // Own setup
    this->sub_to_discovery_ = this->create_subscription<comms::msg::Status>(
        this->discovery_topic_, this->data_qos_,
        std::bind(&Pelican::storeAttendance, this, std::placeholders::_1), this->reentrant_opt_
    );
    this->sub_to_dispatch_ = this->create_subscription<comms::msg::Command>(
        this->dispatch_topic_, this->qos_,
        std::bind(&Pelican::handleCommandReception, this, std::placeholders::_1),
        this->reentrant_opt_
    );
    this->sub_to_locator_ = this->create_subscription<comms::msg::NetworkVertex>(
        this->locator_topic_, this->qos_,
        std::bind(&Pelican::recordCopterPosition, this, std::placeholders::_1), this->reentrant_opt_
    );
    this->pub_to_discovery_ =
        this->create_publisher<comms::msg::Status>(this->discovery_topic_, this->data_qos_);
    this->pub_to_dispatch_ =
        this->create_publisher<comms::msg::Command>(this->dispatch_topic_, this->qos_);
    this->pub_to_locator_ =
        this->create_publisher<comms::msg::NetworkVertex>(this->locator_topic_, this->qos_);

    // Other modules' setup
    this->logger_.initSetup(std::make_shared<rclcpp::Logger>(this->get_logger()), this->getID());
    this->hb_core_.initSetup(&(this->logger_));
    this->el_core_.initSetup(&(this->logger_));
    this->tac_core_.initSetup(&(this->logger_));
    this->unsc_core_.initSetup(&(this->logger_));

    // Parse SDF model
    this->parseModel();

    // Log parameters values
    this->sendLogInfo("Loaded model {} | Agent mass: {}", this->getModel(), this->getMass());

    // Wait and notify presence to other agents
    std::this_thread::sleep_for(std::chrono::seconds(1));
    this->rollCall();

    this->ready_ = true;
    this->sendLogInfo("Node ready!");

    this->becomeFollower();
}

Pelican::~Pelican() {
    this->sendLogDebug("Destructor for agent {}", this->getID());

    // Reset shared pointers
    this->instance_.reset();
}

/********************** Core functionalities ***********************/
void Pelican::parseModel() {
    this->sendLogDebug("Trying to load model {}", this->getModel());
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
        this->sendLogError(
            "Model file {} could not be loaded! Error description: {}", this->getModel(),
            result.description()
        );
        // Abort everything
        throw std::runtime_error("Agent model could not be parsed!");
    }
}

void Pelican::signalHandler(int signum) {
    // Only handle SIGINT
    if (signum == SIGINT) {
        // Stop the thread gracefully
        std::shared_ptr<Pelican> node = getInstance();
        if (node) {
            node->hb_core_.stopService();
            node->el_core_.stopService();
            node->tac_core_.stopService();
            node->unsc_core_.stopService();
        }

        rclcpp::shutdown();
    }
}

void Pelican::rollCall() {
    comms::msg::Status msg;
    msg.agent_id = this->getID();
    msg.status = true;

    this->sendLogInfo("Notifying presence to the fleet");
    this->pub_to_discovery_->publish(msg);
}

void Pelican::storeAttendance(comms::msg::Status::SharedPtr msg) {
    // Cancel timer to avoid parallel executions // CHECK: can be done differently in ROS2?
    cancelTimer(this->netsize_timer_);

    std::lock_guard<std::mutex> lock(this->discovery_mutex_);

    // Exclude this node's own ID, since it doesn't make sense
    if (msg->agent_id != this->getID()) {
        if (std::find_if_not(
                this->discovery_vector_.begin(), this->discovery_vector_.end(),
                [msg, this](const comms::msg::Status& obj) {
                    return (msg->agent_id != obj.agent_id);
                }
            ) == this->discovery_vector_.end()) {
            this->sendLogDebug("Storing discovery msg with id {}", msg->agent_id);
            this->discovery_vector_.push_back(*msg);
        } else {
            this->sendLogDebug("Agent {} already discovered", msg->agent_id);
        }
    } else {
        this->sendLogDebug("Excluding own agent_id from the discovery");
    }

    // (Re)set a timer which logs the inferred network size
    this->netsize_timer_ = this->create_wall_timer(
        this->netsize_timeout_,
        [this]() {
            cancelTimer(this->netsize_timer_); // Execute this only once
            auto net_size = this->getNetworkSize();
            this->sendLogDebug("Discovered network has size {}", net_size);
            this->resizeCopterPositionsVector(net_size);
        },
        this->getReentrantGroup()
    );
}

void Pelican::sharePosition(geometry_msgs::msg::Point pos) { // TODO: check for NAN?
    this->sendLogDebug("Sharing pos: ({:.4f},{:.4f},{:.4f})", pos.x, pos.y, pos.z);
    auto own_id = this->getID();

    this->positions_mutex_.lock();
    // this->copters_positions_[own_id-1] = pos;
    this->positions_mutex_.unlock();

    comms::msg::NetworkVertex msg;
    msg.id = own_id;
    msg.position = pos;

    this->pub_to_locator_->publish(msg);
}

void Pelican::recordCopterPosition(const comms::msg::NetworkVertex msg) {
    // TODO: check for nan
    std::lock_guard<std::mutex> lock(this->positions_mutex_);
    this->sendLogDebug(
        "Copter {} shared position: ({:.4f},{:.4f},{:.4f})", msg.id, msg.position.x, msg.position.y,
        msg.position.z
    );
    if (this->copters_positions_.size() > 0) {
        this->copters_positions_[msg.id - 1] = msg.position;
    } else {
        this->sendLogDebug("Ignoring because I don't know the fleet size, yet");
    }
}

// CHECK: add how to delete? or use another function?
void Pelican::resizeCopterPositionsVector(unsigned int new_size) {
    this->sendLogDebug("Resizing copter_positions_ vector to size {}", new_size);
    std::lock_guard<std::mutex> lock(this->positions_mutex_);

    int size_diff = this->copters_positions_.size() - new_size;
    // Incrementing vector
    if (size_diff < 0) { // TODO: invert logic
        this->sendLogDebug("Increasing it, adding {} elements", abs(size_diff));
        for (int i = 0; i < abs(size_diff); i++) {
            this->copters_positions_.push_back(NAN_point);
        }
    } else {
        this->sendLogDebug("Downsizing position vector from length {}", size_diff + new_size);
    }
}
