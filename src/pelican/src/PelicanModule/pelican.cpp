/**
 * @file pelican.cpp
 * @author Simone Vollaro (slim71sv@gmail.com)
 * @brief Main methods of the PelicanModule object.
 * @version 1.0.0
 * @date 2024-11-17
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "PelicanModule/pelican.hpp"

// Initialize the static instance pointer to a weak pointer
std::weak_ptr<Pelican> Pelican::instance_;

/************************** Ctors/Dctors ***************************/
/**
 * @brief Construct a new Pelican object.
 *
 */
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
    this->ballot_exclusive_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->ballot_opt_.callback_group = this->ballot_exclusive_group_;

    // Set everything up
    this->enableSystem();

    // Parse SDF model
    this->parseModel();

    // Log parameters values
    this->sendLogInfo("Loaded model {} | Agent mass: {}", this->getModel(), this->getMass());
    this->sendLogInfo("Parameters: roi: {}", this->roi_);

    this->ready_ = true;
    this->sendLogInfo("Node ready!");

    this->becomeFollower();
}

/**
 * @brief Destroy the Pelican object.
 *
 */
Pelican::~Pelican() {
    this->sendLogDebug("Destructor for agent {}", this->getID());

    // Notify condition variables to avoid deadlocks
    rendezvous_handled_cv_.notify_all();
    formation_handled_cv_.notify_all();

    // Stop and reset all ROS2-related services, subscriptions, and timers
    // Clean up action servers
    resetSharedPointer(teleopdata_server_);

    // Stop all service clients
    resetSharedPointer(fleetinfo_client_);
    resetSharedPointer(cargo_attachment_client_);
    resetSharedPointer(des_pos_client_);
    resetSharedPointer(form_reached_client_);

    // Stop all services
    resetSharedPointer(fleetinfo_server_);
    resetSharedPointer(des_pos_server_);
    resetSharedPointer(form_reached_server_);

    // Cancel and reset all subscriptions and publishers
    resetSharedPointer(sub_to_dispatch_);
    resetSharedPointer(pub_to_dispatch_);
    resetSharedPointer(sub_to_locator_);
    resetSharedPointer(pub_to_locator_);
    resetSharedPointer(sub_to_formation_);
    resetSharedPointer(pub_to_formation_);
    resetSharedPointer(sub_to_sync_);
    resetSharedPointer(pub_to_sync_);

    // Clear the callback groups
    resetSharedPointer(reentrant_group_);
    resetSharedPointer(timer_exclusive_group_);
    resetSharedPointer(offboard_exclusive_group_);
    resetSharedPointer(rendezvous_exclusive_group_);
    resetSharedPointer(formation_exclusive_group_);
    resetSharedPointer(formation_timer_group_);
    resetSharedPointer(target_exclusive_group_);
    resetSharedPointer(height_exclusive_group_);
    resetSharedPointer(check_exclusive_group_);
    resetSharedPointer(p2p_exclusive_group_);
    resetSharedPointer(ballot_exclusive_group_);

    // Clear internal data structures
    copters_positions_.clear();
    rpcs_vector_.clear();
    discovery_vector_.clear();
    dispatch_vector_.clear();
    agents_in_formation_.clear();

    // Reset shared pointers
    this->instance_.reset();
}

/********************** Core functionalities ***********************/
/**
 * @brief Parses the agent's model file to extract relevant parameters such as mass.
 *
 * The method uses an XML parser to load and parse the specified model file. If the model file
 * is successfully parsed, the agent's mass is extracted from the `<inertial>` tag under the
 * `base_link` node. If parsing fails, an error is logged, and an exception is thrown.
 *
 * @throws std::runtime_error if the model file cannot be parsed.
 */
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

/**
 * @brief Handles the termination signal (SIGINT) to gracefully stop the node's services.
 *
 * Stops core services associated with the Pelican node and shuts down the ROS 2 framework.
 * Only handles SIGINT signals.
 *
 * @param signum The signal number, expected to be SIGINT.
 */
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

/**
 * @brief Initializes the system by setting up ROS 2 publishers, subscribers, service servers,
 *        and clients, as well as other core modules.
 *
 * The method subscribes to topics for locator, dispatch, and formation data, and publishes
 * corresponding topics. It also sets up service servers and clients for inter-agent communication.
 */
void Pelican::enableSystem() {
    // Subscribers
    this->sub_to_locator_ = this->create_subscription<comms::msg::NetworkVertex>(
        this->locator_topic_, this->data_qos_,
        std::bind(&Pelican::handleCopterInfo, this, std::placeholders::_1), this->reentrant_opt_
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
}

/**
 * @brief Transitions the node to a failure mode when a critical failure is detected.
 *
 * The method stops all core services, resets pointers to ROS 2 communication entities, logs
 * the failure, and notifies other agents of the node's inactive status.
 */
void Pelican::transitionToFailureMode() {
    this->setRole(tbd);
    this->sendLogWarning("Failsafe detected! Transitioning to dormient mode...");
    this->hb_core_.stopService();
    this->tac_core_.stopService();
    this->unsc_core_.stopService();
    this->el_core_.stopService();
    this->failureMode();
}

/**
 * @brief Cleans up resources and notifies the system of the node's failure state.
 *
 * Resets all shared pointers for subscriptions, publications, and services. Broadcasts an
 * inactive status message to the locator topic and logs the failure.
 */
void Pelican::failureMode() {
    resetSharedPointer(this->sub_to_dispatch_);
    resetSharedPointer(this->sub_to_formation_);
    resetSharedPointer(this->sub_to_locator_);
    resetSharedPointer(this->sub_to_sync_);
    resetSharedPointer(this->pub_to_dispatch_);
    resetSharedPointer(this->pub_to_formation_);
    resetSharedPointer(this->pub_to_sync_);
    resetSharedPointer(this->teleopdata_server_);
    resetSharedPointer(this->fleetinfo_server_);
    resetSharedPointer(this->fleetinfo_client_);
    resetSharedPointer(this->cargo_attachment_client_);
    resetSharedPointer(this->des_pos_server_);
    resetSharedPointer(this->des_pos_client_);
    resetSharedPointer(this->form_reached_server_);
    resetSharedPointer(this->form_reached_client_);

    this->sendLogWarning("Notifying deactivation due to failure");
    comms::msg::NetworkVertex msg;
    msg.agent_id = this->getID();
    msg.status = false;
    this->pub_to_locator_->publish(msg);

    resetSharedPointer(this->pub_to_locator_);
}

/**
 * @brief Recovers the node from failure mode, reinitializing the system and transitioning
 *        the node to the follower role.
 *
 * The method re-enables system components, sets the node's readiness status to `true`, and
 * assigns it a follower role.
 */
void Pelican::recoverFromFailure() {
    this->sendLogInfo("Rebooting system since failure mode has ended");
    this->enableSystem();
    this->ready_ = true;

    this->sendLogInfo("Node ready again!");
    this->becomeFollower();
}

/************************** Data exchange **************************/
void Pelican::handleCopterInfo(const comms::msg::NetworkVertex::SharedPtr msg) {
    if (msg->status) {
        this->recordCopterPosition(msg);
        this->storeAttendance(msg->agent_id);
    } else {
        this->unsc_core_.reactToFailureNotification(msg->agent_id);
        this->sendLogWarning(
            "Agent {} notified a malfunction: excluding it from future tasks...", msg->agent_id
        );

        std::lock_guard lock_pos(this->positions_mutex_);
        this->copters_positions_.erase(msg->agent_id);

        std::lock_guard lock_discovery(this->discovery_mutex_);
        auto position = std::find(
            this->discovery_vector_.begin(), this->discovery_vector_.end(), msg->agent_id
        );
        if (position != this->discovery_vector_.end())
            this->discovery_vector_.erase(position);

        std::lock_guard lock_form(this->form_reached_mutex_);
        position = std::find(
            this->agents_in_formation_.begin(), this->agents_in_formation_.end(), msg->agent_id
        );
        if (position != this->agents_in_formation_.end())
            this->agents_in_formation_.erase(position);
    }
}

void Pelican::storeAttendance(unsigned int agent_id) {
    std::lock_guard lock(this->discovery_mutex_);

    // Exclude this node's own ID, since it doesn't make sense
    if (agent_id == this->getID()) {
        this->sendLogDebug("Excluding own agent_id from the discovery");
        return;
    }

    if (std::find_if_not(
            this->discovery_vector_.begin(), this->discovery_vector_.end(),
            [agent_id](unsigned int discovered) {
                return (agent_id != discovered);
            }
        ) == this->discovery_vector_.end()) {
        this->sendLogDebug("Storing discovery msg with id {}", agent_id);
        this->discovery_vector_.push_back(agent_id);

    } else {
        this->sendLogDebug("Agent {} already discovered", agent_id);
        return;
    }

    this->sendLogDebug("Network size for now: {}", this->discovery_vector_.size() + 1);
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

/**
 * @brief Handles the accepted teleoperation data and spins up a new thread for processing.
 *
 * When a goal to handle teleoperation data is accepted, the method immediately
 * dispatches the handling to a separate thread to avoid blocking the main execution flow.
 *
 * @param goal_handle The handle for the goal that was accepted.
 */
void Pelican::handleAcceptedTeleopData(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<comms::action::TeleopData>> goal_handle
) {
    this->sendLogDebug("Received accepted goal to handle");
    // This needs to return quickly to avoid blocking the executor,
    // so spin up a new thread for the actual handling
    std::thread {std::bind(&Pelican::rogerWillCo, this, std::placeholders::_1), goal_handle}.detach(
    );
}

/**
 * @brief Generates a CargoLinkage request to attach or detach cargo.
 *
 * The method builds a request to the `CargoLinkage` service based on the provided
 * `attach` flag. It checks the service availability and sends the request asynchronously.
 *
 * @param attach Boolean flag indicating whether to attach or detach the cargo.
 */
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
            rcutils_reset_error(); // Reset the error after handling
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

/**
 * @brief Handles the response from the CargoLinkage service.
 *
 * This method checks the response from the cargo attachment service and logs the result.
 * It determines whether the cargo attachment operation succeeded or failed.
 *
 * @param future The future representing the response from the CargoLinkage service.
 */
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

/**
 * @brief Stores the desired position for the current agent based on the FormationDesired message.
 *
 * This method extracts the desired position for the agent from the incoming message
 * and stores it in the agent's internal state.
 *
 * @param msg The FormationDesired message containing the desired positions for all agents.
 */
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

/**
 * @brief Requests the desired position of a neighboring agent.
 *
 * The method creates a client to request the desired position of a neighboring agent
 * and waits for a response. It retries the request if the service is unavailable within
 * a certain timeout period.
 *
 * @param id The ID of the neighboring agent whose desired position is being requested.
 */
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
            rcutils_reset_error(); // Reset the error after handling
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

/**
 * @brief Handles the result of a FleetInfo request, specifically the desired position of a
 * neighbor.
 *
 * This method processes the response from a neighbor's service containing their desired
 * position and updates the agent's internal knowledge of the neighbor's position.
 *
 * @param future The future representing the response from the FleetInfo service.
 */
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

/**
 * @brief Records an agent as part of the formation when the FormationReached request is received.
 *
 * This method is executed by the leader and registers an agent in the formation if it is
 * not already recorded. Once all agents are recorded, it confirms that the formation is
 * achieved.
 *
 * @param request The FormationReached request containing the agent ID.
 * @param response The response to the request (unused in this method).
 */
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

/**
 * @brief Notifies that the agent is in formation by sending a FormationReached request.
 *
 * This method is executed by the followers to notify the leader that they have reached
 * their desired formation position.
 */
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
            rcutils_reset_error(); // Reset the error after handling
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

/****************************** Other ******************************/
/**
 * @brief Clears the list of agents currently in formation.
 *
 * The method acquires a lock to ensure thread safety and empties the
 * `agents_in_formation_` container.
 */
void Pelican::emptyFormationResults() {
    std::lock_guard lock(this->form_reached_mutex_);
    this->agents_in_formation_.clear();
}

/**
 * @brief Handles the completion signal for a fleet operation.
 *
 * When a synchronization message is received, it updates the internal state to
 * store the last completed operation, based on the provided `completed_op` field
 * in the message.
 *
 * @param msg The FleetSync message containing the completed operation ID.
 */
void Pelican::handleSyncCompletedOp(comms::msg::FleetSync msg) {
    this->sendLogDebug("Received sync signal for operation {}", msg.completed_op);
    this->unsc_core_.setLastCompletedOperation(msg.completed_op);
}

/**
 * @brief Publishes a message indicating that a fleet operation has been completed.
 *
 * Verifies the validity of the operation ID before publishing a `FleetSync` message
 * to notify other nodes of the completed operation. Logs a warning if the operation
 * ID is unrecognized.
 *
 * @param cmd The operation ID to mark as completed.
 */
void Pelican::syncCompletedOp(uint32_t cmd) {
    if (cmd > comms::msg::FleetSync::CARGO_DETACHED) {
        this->sendLogWarning("Operation {} not recognized!", cmd);
        return;
    }
    auto msg = comms::msg::FleetSync();
    msg.completed_op = cmd;
    this->pub_to_sync_->publish(msg);
}
