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
    declare_parameter("model", "");             // default to ""
    declare_parameter("id", 0);                 // default to 0
    declare_parameter("roi", 2.0);              // default to 2.0
    declare_parameter("collision_radius", 1.0); // default to 1.0

    // Get parameters values and store them
    get_parameter("model", this->model_);
    get_parameter("id", this->id_);
    get_parameter("roi", this->roi_);
    get_parameter("collision_radius", this->collision_radius_);

    // Setting up callback groups
    this->reentrant_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    this->reentrant_opt_.callback_group = this->reentrant_group_;
    this->timer_exclusive_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->timer_exclusive_opt_.callback_group = this->timer_exclusive_group_;
    this->offboard_exclusive_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->offboard_exclusive_opt_.callback_group = this->offboard_exclusive_group_;
    this->rendezvous_exclusive_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->rendezvous_exclusive_opt_.callback_group = this->rendezvous_exclusive_group_;

    // Own setup
    this->sub_to_locator_ = this->create_subscription<comms::msg::NetworkVertex>(
        this->locator_topic_, this->data_qos_,
        std::bind(&Pelican::storeCopterInfo, this, std::placeholders::_1), this->reentrant_opt_
    );
    this->sub_to_dispatch_ = this->create_subscription<comms::msg::Command>(
        this->dispatch_topic_, this->qos_,
        std::bind(&Pelican::handleCommandReception, this, std::placeholders::_1),
        this->reentrant_opt_
    );
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
    // Thanks to this package's hook, the list of paths this searches into
    // is automatically extended to include the actual folder being used
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

void Pelican::storeAttendance(comms::msg::NetworkVertex::SharedPtr msg) {
    std::lock_guard lock(this->discovery_mutex_);

    // Exclude this node's own ID, since it doesn't make sense
    if (msg->agent_id == this->getID()) {
        this->sendLogDebug("Excluding own agent_id from the discovery");
        return;
    }

    if (std::find_if_not(
            this->discovery_vector_.begin(), this->discovery_vector_.end(),
            [msg, this](const comms::msg::NetworkVertex& obj) {
                return (msg->agent_id != obj.agent_id);
            }
        ) == this->discovery_vector_.end()) {
        this->sendLogDebug("Storing discovery msg with id {}", msg->agent_id);
        this->discovery_vector_.push_back(*msg);

    } else {
        this->sendLogDebug("Agent {} already discovered", msg->agent_id);
        return;
    }

    this->sendLogDebug("Setting discovery timer");
    // (Re)set a timer which logs the inferred network size
    this->netsize_timer_ = this->create_wall_timer(
        this->netsize_timeout_,
        [this]() {
            cancelTimer(this->netsize_timer_); // Execute this only once
            auto net_size = this->getNetworkSize();
            this->sendLogDebug("Discovered network has size {}", net_size);
            this->resizeCopterPositionsVector(net_size);
        },
        this->getTimerExclusiveGroup()
    );
}

void Pelican::storeCopterInfo(const comms::msg::NetworkVertex::SharedPtr msg) {
    this->recordCopterPosition(msg);
    this->storeAttendance(msg);
}

void Pelican::sharePosition(geometry_msgs::msg::Point pos) {
    this->sendLogDebug("Sharing pos: {}", pos);

    comms::msg::NetworkVertex msg;
    msg.agent_id = this->getID();
    msg.status = true;
    msg.position = pos;

    this->pub_to_locator_->publish(msg);
}

void Pelican::recordCopterPosition(comms::msg::NetworkVertex::SharedPtr msg) {
    std::lock_guard lock(this->positions_mutex_);
    this->sendLogDebug("Copter {} shared position: {}", msg->agent_id, msg->position);
    if (!this->copters_positions_.empty()) {
        this->copters_positions_[msg->agent_id - 1] = msg->position;
    } else {
        this->sendLogDebug("Ignoring because I don't know the fleet size, yet");
    }
}

void Pelican::resizeCopterPositionsVector(unsigned int new_size, unsigned int lost_id) {
    this->sendLogDebug("Resizing copter_positions_ vector to size {}", new_size);
    std::lock_guard lock(this->positions_mutex_);

    int size_diff = new_size - this->copters_positions_.size();
    // Incrementing vector
    if (size_diff > 0) {
        this->sendLogDebug("Increasing it, adding {} elements", size_diff);
        for (int i = 0; i < size_diff; i++) {
            this->copters_positions_.push_back(NAN_point);
        }
    } else {
        // Record a NAN position, which will be ignored
        this->sendLogWarning("Agent with ID {} lost connection to the network", lost_id);
        if (lost_id == UINT_MAX) {
            this->sendLogWarning("Invalid 'downsizing'");
            return;
        }
        if (lost_id < this->copters_positions_.size()) {
            this->copters_positions_[lost_id] = NAN_point;
        } else {
            this->sendLogWarning("Trying to lose an ID greater than vector size");
        }
    }
}

rclcpp_action::GoalResponse Pelican::
    handleTeleopDataGoal(const rclcpp_action::GoalUUID&, std::shared_ptr<const comms::action::TeleopData::Goal>) {
    this->sendLogDebug("Received goal request from user");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
Pelican::handleTeleopDataCancellation(const std::shared_ptr<
                                      rclcpp_action::ServerGoalHandle<comms::action::TeleopData>>) {
    this->sendLogWarning("Received goal cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Pelican::handleAcceptedTeleopData(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<comms::action::TeleopData>> goal_handle
) {
    this->sendLogDebug("Received accepted goal to handle");
    // This needs to return quickly to avoid blocking the executor,
    // so spin up a new thread for the actual handling
    std::thread {std::bind(&Pelican::rogerWillCo, this, std::placeholders::_1), goal_handle}.detach(
    );
}
