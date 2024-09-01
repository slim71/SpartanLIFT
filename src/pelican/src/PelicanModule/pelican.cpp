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
    declare_parameter("roi", 2.0);  // default to 2.0

    // Get parameters values and store them
    get_parameter("model", this->model_);
    get_parameter("id", this->id_);
    get_parameter("roi", this->roi_);

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
    this->formation_exclusive_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->formation_exclusive_opt_.callback_group = this->formation_exclusive_group_;
    this->trigger_exclusive_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->trigger_opt_.callback_group = this->trigger_exclusive_group_;
    this->trigger_handle_exclusive_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->trigger_handle_opt_.callback_group = this->trigger_handle_exclusive_group_;
    this->target_exclusive_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->target_opt_.callback_group = this->target_exclusive_group_;
    this->height_exclusive_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->height_opt_.callback_group = this->height_exclusive_group_;
    this->check_exclusive_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->check_opt_.callback_group = this->check_exclusive_group_;
    this->p2p_exclusive_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->p2p_opt_.callback_group = this->p2p_exclusive_group_;

    // Subscribers
    this->sub_to_locator_ = this->create_subscription<comms::msg::NetworkVertex>(
        this->locator_topic_, this->data_qos_,
        std::bind(&Pelican::storeCopterInfo, this, std::placeholders::_1), this->reentrant_opt_
    );
    this->sub_to_dispatch_ = this->create_subscription<comms::msg::Command>(
        this->dispatch_topic_, this->qos_,
        std::bind(&Pelican::handleCommandReception, this, std::placeholders::_1),
        this->reentrant_opt_
    );
    this->sub_to_formation_ = this->create_subscription<comms::msg::FormationDesired>(
        this->formation_topic_, this->qos_,
        std::bind(&Pelican::storeDesiredPosition, this, std::placeholders::_1),
        this->formation_exclusive_opt_
    );
    // Publishers
    this->pub_to_dispatch_ =
        this->create_publisher<comms::msg::Command>(this->dispatch_topic_, this->qos_);
    this->pub_to_locator_ =
        this->create_publisher<comms::msg::NetworkVertex>(this->locator_topic_, this->qos_);
    this->pub_to_formation_ =
        this->create_publisher<comms::msg::FormationDesired>(this->formation_topic_, this->qos_);
    // Service clients
    this->cargo_attachment_client_ =
        this->create_client<comms::srv::CargoLinkage>("attachment_service");
    // Service servers
    this->des_pos_server_ = this->create_service<comms::srv::FleetInfo>(
        "des_pos_service_" + std::to_string(this->getID()),
        std::bind(
            &Pelican::shareDesiredPosition, this, std::placeholders::_1, std::placeholders::_2
        ),
        rmw_qos_profile_services_default, this->getReentrantGroup()
    );

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
    this->sendLogInfo("Parameters: roi: {}", this->roi_);

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
}

void Pelican::storeCopterInfo(const comms::msg::NetworkVertex::SharedPtr msg) {
    this->recordCopterPosition(msg);
    this->storeAttendance(msg);
}

// Publisher to "/fleet/network"
void Pelican::sharePosition(geometry_msgs::msg::Point pos) {
    this->sendLogDebug("Sharing pos: {}", pos);

    comms::msg::NetworkVertex msg;
    msg.agent_id = this->getID();
    msg.status = true;
    msg.position = pos;

    this->pub_to_locator_->publish(msg);
}

// FleetInfo request handler - Neighbor desired position
void Pelican::shareDesiredPosition(
    const std::shared_ptr<comms::srv::FleetInfo::Request>,
    std::shared_ptr<comms::srv::FleetInfo::Response> response
) {
    auto my_pos = this->getDesiredPosition();
    response->formation = true;
    response->target = my_pos;
    response->agent_id = this->getID();
}

void Pelican::recordCopterPosition(comms::msg::NetworkVertex::SharedPtr msg) {
    std::lock_guard lock(this->positions_mutex_);
    this->sendLogDebug("Copter {} shared position: {}", msg->agent_id, msg->position);
    this->copters_positions_[msg->agent_id] = msg->position;
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

// CargoLinkage request generator
void Pelican::cargoAttachment(bool attach) {
    // Create request
    auto request = std::make_shared<comms::srv::CargoLinkage::Request>();
    std::string model_path = this->getModel();
    std::regex rgx(R"(/([^/]+)/model\.sdf)");
    std::smatch match;
    std::string model_name;

    if (std::regex_search(model_path, match, rgx)) {
        model_name = match[1];
    } else {
        this->sendLogWarning("No match found");
        return;
    }
    request->set__leader_id(this->getID()).set__model(model_name);
    request->set__attach(attach);

    // Search for a second, then log and search again if needed
    unsigned int total_search_time = 0;
    std::string service_name = this->cargo_attachment_client_->get_service_name();
    while (!this->cargo_attachment_client_->wait_for_service(
               std::chrono::seconds(constants::SEARCH_SERVER_STEP_SECS)
           ) &&
           total_search_time < constants::MAX_SEARCH_TIME_SECS) {
        if (!rclcpp::ok()) {
            this->sendLogError(
                "Client interrupted while waiting for the {} service. Terminating...", service_name
            );
            return;
        }

        this->sendLogDebug("Service {} not available; waiting some more...", service_name);
        total_search_time += constants::SEARCH_SERVER_STEP_SECS;
    };

    if (total_search_time >= constants::MAX_SEARCH_TIME_SECS) {
        this->sendLogWarning("The {} server seems to be down. Please try again.", service_name);
        return;
    }
    this->sendLogDebug("{} server available", service_name);

    // Send request
    auto async_request_result = this->cargo_attachment_client_->async_send_request(
        request, std::bind(&Pelican::checkCargoAttachment, this, std::placeholders::_1)
    );
    // Check if request was accepted and cleanup if not (not to waste memory)
    auto future_status =
        async_request_result.wait_for(std::chrono::seconds(constants::SERVICE_FUTURE_WAIT_SECS));
    if (!async_request_result.valid() || (future_status != std::future_status::ready)) {
        this->sendLogWarning("Failed to receive confirmation from the {} server!", service_name);
        this->cargo_attachment_client_->prune_pending_requests();
        return;
    }
}

// CargoLinkage response handler
void Pelican::checkCargoAttachment(rclcpp::Client<comms::srv::CargoLinkage>::SharedFuture future) {
    // Wait for the specified amount or until the result is available
    this->sendLogDebug("Getting response...");
    auto status = future.wait_for(std::chrono::seconds(constants::SERVICE_FUTURE_WAIT_SECS));

    if (status != std::future_status::ready) {
        this->sendLogDebug("Service not ready yet...");
        return;
    }

    auto response = future.get();
    if (response->done) {
        this->sendLogDebug("Cargo attachment completed");
        // this->syncCompletedOp();
    } else
        this->sendLogWarning("Issues with cargo attachment!");
}

// Receiver of FormationDesired messages
void Pelican::storeDesiredPosition(const comms::msg::FormationDesired msg) {
    for (auto&& pos : msg.des_positions) {
        if (pos.agent_id == this->getID()) {
            std::lock_guard lock(this->formation_mutex_);
            this->des_formation_pos_ = pos.position;
            this->sendLogDebug("Received desired formation position {}", pos.position);
            return;
        }
    }
    this->sendLogDebug("Could not find my desired formation position in the received message");
}

// FleetInfo request generator - Neighbor desired position request
void Pelican::askDesPosToNeighbor(unsigned int id) {
    this->sendLogDebug("Asking neighbor {} its desired position...", id);
    this->des_pos_client_ = this->create_client<comms::srv::FleetInfo>(
        "des_pos_service_" + std::to_string(id), rmw_qos_profile_services_default,
        this->getReentrantGroup()
    );

    // Search for a second, then log and search again if needed
    unsigned int total_search_time = 0;
    std::string service_name = this->des_pos_client_->get_service_name();
    while (!this->des_pos_client_->wait_for_service(
               std::chrono::seconds(constants::SEARCH_SERVER_STEP_SECS)
           ) &&
           total_search_time < constants::MAX_SEARCH_TIME_SECS) {
        if (!rclcpp::ok()) {
            this->sendLogError(
                "Client interrupted while waiting for {} server. Terminating...", service_name
            );
            return;
        }

        this->sendLogDebug("Service {} not available; waiting some more...", service_name);
        total_search_time += constants::SEARCH_SERVER_STEP_SECS;
    };

    if (total_search_time >= constants::MAX_SEARCH_TIME_SECS) {
        this->sendLogWarning("The {} server seems to be down. Please try again.", service_name);
        return;
    }

    this->sendLogDebug("{} server available", service_name);
    // Send request
    auto request = std::make_shared<comms::srv::FleetInfo::Request>();
    auto async_request_result = this->des_pos_client_->async_send_request(
        request, std::bind(&Pelican::storeNeighborDesPos, this, std::placeholders::_1)
    );
    // Check if request was accepted and cleanup if not (not to waste memory)
    auto future_status =
        async_request_result.wait_for(std::chrono::seconds(constants::SERVICE_FUTURE_WAIT_SECS));
    if (!async_request_result.valid() || (future_status != std::future_status::ready)) {
        this->sendLogWarning(
            "Failed to receive confirmation from the {} server (desired position)!", service_name
        );
        this->des_pos_client_->prune_pending_requests();
        return;
    }
}

// Result handler of FleetInfo data - Neighbor desired position
void Pelican::storeNeighborDesPos(rclcpp::Client<comms::srv::FleetInfo>::SharedFuture future) {
    // Wait for the specified amount or until the result is available
    this->sendLogDebug("Getting neighbor position...");
    auto status = future.wait_for(std::chrono::seconds(constants::SERVICE_FUTURE_WAIT_SECS));

    if (status == std::future_status::ready) {
        this->sendLogDebug("Neighbor des_pos service ready...");
        auto response = future.get();
        if (response) {
            this->sendLogDebug(
                "Received from neighbor {} desired position {}", response->agent_id,
                response->target
            );
            this->setNeighborPosition(response->target);
        } else {
            this->sendLogWarning("Received an empty response from the neighbor's service");
        }
    } else {
        this->sendLogDebug("Neighbor des_pos service not ready yet...");
    }

    this->initiateUnblockFormation();
}

// FormationReached request handler - executed only by the leader
void Pelican::
    recordAgentInFormation(const std::shared_ptr<comms::srv::FormationReached::Request> request, std::shared_ptr<comms::srv::FormationReached::Response>) {
    this->sendLogDebug("Checking if agent has to be recorded");
    std::lock_guard lock(this->form_reached_mutex_);
    if (std::find(
            this->agents_in_formation_.begin(), this->agents_in_formation_.end(), request->agent_id
        ) == this->agents_in_formation_.end()) {
        this->sendLogDebug("New agent in formation to be registered: {}", request->agent_id);
        this->agents_in_formation_.push_back(request->agent_id);
    }

    if (this->agents_in_formation_.size() == this->getNetworkSize() - 1) {
        this->sendLogDebug(
            "Confirming formation is achieved (size: {}, network: {})",
            this->agents_in_formation_.size(), this->getNetworkSize()
        );
        this->setFormationAchieved();
    }
}

// FormationReached request generator - executed by the followers
void Pelican::notifyAgentInFormation() {
    std::string service_name = this->form_reached_client_->get_service_name();
    this->sendLogDebug("Notifying I'm in formation...");

    // Create request
    auto request = std::make_shared<comms::srv::FormationReached::Request>();
    request->set__agent_id(this->getID());

    // Search for a second, then log and search again if needed
    unsigned int total_search_time = 0;
    while (!this->form_reached_client_->wait_for_service(
               std::chrono::seconds(constants::SEARCH_SERVER_STEP_SECS)
           ) &&
           total_search_time < constants::MAX_SEARCH_TIME_SECS) {
        if (!rclcpp::ok()) {
            this->sendLogError(
                "Client interrupted while waiting for the {} server. Terminating...", service_name
            );
            return;
        }
        this->sendLogDebug("Service {} not available; waiting some more...", service_name);
        total_search_time += constants::SEARCH_SERVER_STEP_SECS;
    };

    if (total_search_time >= constants::MAX_SEARCH_TIME_SECS) {
        this->sendLogWarning("The {} server seems to be down. Please try again.", service_name);
        return;
    }

    this->sendLogDebug("{} server available", service_name);
    // Send request
    auto async_request_result = this->form_reached_client_->async_send_request(request);
    // Check if request was accepted and cleanup if not (not to waste memory)
    auto future_status =
        async_request_result.wait_for(std::chrono::seconds(constants::SERVICE_FUTURE_WAIT_SECS));
    if (!async_request_result.valid() || (future_status != std::future_status::ready)) {
        this->sendLogWarning("Failed to receive confirmation from the {} server!", service_name);
        this->form_reached_client_->prune_pending_requests();
        return;
    }
}

void Pelican::syncCompletedOp(uint32_t cmd) {
    if (cmd > comms::msg::FleetSync::CARGO_DETACHED) {
        this->sendLogWarning("Operation {} not recognized!", cmd);
        return;
    }
    auto msg = comms::msg::FleetSync();
    msg.completed_op = cmd;
    this->pub_to_sync_->publish(msg);
}

void Pelican::handleSyncCompletedOp(comms::msg::FleetSync msg) {
    this->sendLogDebug("Received sync signal for operation {}", msg.completed_op);
    this->unsc_core_.setLastCompletedOperation(msg.completed_op);
}

void Pelican::emptyFormationResults() {
    std::lock_guard lock(this->form_reached_mutex_);
    this->agents_in_formation_.clear();
}
