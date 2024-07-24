#include "PelicanModule/pelican.hpp"

/*************** Service- and action server- related ***************/
// This is only executed by the leader, receiver of TeleopData goals, to handle an accepted goal
void Pelican::rogerWillCo(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<comms::action::TeleopData>> goal_handle
) {
    const auto request = goal_handle->get_goal();
    this->sendLogDebug(
        "Handling goal with request: presence={} takeoff={}, landing={} retrieval={}",
        request->presence, request->takeoff, request->landing, request->retrieval
    );

    this->last_goal_mutex_.lock();
    this->last_goal_handle_ = goal_handle;
    this->last_goal_mutex_.unlock();

    // General all-purpose init
    auto response = std::make_shared<comms::action::TeleopData::Result>();
    response->leader_id = this->getID();
    response->presence = false;
    response->taking_off = false;
    response->landing = false;
    response->retrieval = false;
    response->dropoff = false;

    // Make sure the result flag is not set
    this->unsetLastCmdStatus();

    this->sendLogDebug(
        "RogerRoger present: {}, takeoff: {}, landing: {}, retrieval: {}, dropoff: {}",
        request->presence, request->takeoff, request->landing, request->retrieval, request->dropoff
    );

    // Handling leader ID request
    if (request->presence) {
        this->sendLogDebug("Notifying I'm the leader to the user!");
        response->presence = true;
        goal_handle->succeed(response);
    }

    // Handling takeoff command
    if (request->takeoff) {
        response->taking_off = true;

        if (!this->isFlying()) {
            this->sendLogDebug("Notifying start of takeoff operations!");

            std::thread takeoff_thread(&Pelican::handleCommandDispatch, this, TAKEOFF_COMMAND);
            takeoff_thread.join();
            if (this->isLastCmdExecuted()) {
                this->sendLogInfo("Fleet has taken off!");
                goal_handle->succeed(response);
            } else {
                goal_handle->abort(response);
            }
        } else {
            this->sendLogWarning("The fleet should already be flying!");
            goal_handle->abort(response);
        }
    }

    // Handling landing command
    if (request->landing) {
        response->landing = true;

        if (this->isFlying()) {
            this->sendLogDebug("Notifying start of landing operations!");

            std::thread landing_thread(&Pelican::handleCommandDispatch, this, LANDING_COMMAND);
            landing_thread.join();
            if (this->isLastCmdExecuted()) {
                this->sendLogInfo("Fleet has landed!");
                goal_handle->succeed(response);
            } else {
                goal_handle->abort(response);
            }
        } else {
            this->sendLogWarning("The fleet is not flying!");
            goal_handle->abort(response);
        }
    }

    // Handling payload's initial position notification
    if (request->retrieval) {
        response->retrieval = true;

        if (!this->isCarrying()) {
            this->sendLogDebug("Notifying start of payload extraction operations!");
            this->sendLogInfo(
                "Payload is at {}",
                geometry_msgs::msg::Point().set__x(request->x).set__y(request->y).set__z(request->z)
            );

            this->commenceSetActualTargetHeight(request->z);
            this->initiateSetTargetPosition(
                geometry_msgs::msg::Point().set__x(request->x).set__y(request->y).set__z(request->z)
            );

            std::thread retrieval_thread(&Pelican::handleCommandDispatch, this, RENDEZVOUS);
            retrieval_thread.join();
            if (this->isLastCmdExecuted()) {
                this->sendLogInfo("Rendezvous initiated!");
                goal_handle->succeed(response);
            } else {
                goal_handle->abort(response);
            }
        } else {
            this->sendLogWarning("The payload is already carrying the payload!");
            goal_handle->abort(response);
        }
    }
}

// Leader-side; Receiver of FleetInfo requests - Target position
void Pelican::targetNotification(
    const std::shared_ptr<comms::srv::FleetInfo::Request>,
    const std::shared_ptr<comms::srv::FleetInfo::Response> response
) {
    auto maybe_pos = this->requestTargetPosition();
    double target_height = this->requestActualTargetHeight();
    if (maybe_pos) {
        geometry_msgs::msg::Point pos = maybe_pos.value();
        response->target.x = pos.x;
        response->target.y = pos.y;
        response->target.z = target_height;
    }
}

// Non-leader side; FleetInfo request generator - Target position
void Pelican::rendezvousFleet() {
    // The leader has already set the target position before calling this
    if (!this->isLeader()) {
        // Search for a second, then log and search again if needed
        unsigned int total_search_time = 0;
        while (!this->fleetinfo_client_->wait_for_service(
                   std::chrono::seconds(constants::SEARCH_LEADER_STEP_SECS)
               ) &&
               total_search_time < constants::MAX_SEARCH_TIME_SECS) {
            if (!rclcpp::ok()) {
                this->sendLogError("Client interrupted while waiting for service. Terminating...");
                return;
            }

            this->sendLogDebug("Service not available; waiting some more...");
            total_search_time += constants::SEARCH_LEADER_STEP_SECS;
        };

        if (total_search_time >= constants::MAX_SEARCH_TIME_SECS) {
            this->sendLogWarning("The server seems to be down. Please try again.");
            this->unsetAndNotifyRendezvousHandled();
            return;
        }
        this->sendLogDebug("Rendezvous server available");

        // Send request
        auto request = std::make_shared<comms::srv::FleetInfo::Request>();
        auto async_request_result = this->fleetinfo_client_->async_send_request(
            request, std::bind(&Pelican::processLeaderResponse, this, std::placeholders::_1)
        );
        // Check if request was accepted and cleanup if not (not to waste memory)
        auto future_status =
            async_request_result.wait_for(std::chrono::seconds(constants::SERVICE_FUTURE_WAIT_SECS)
            );
        if (!async_request_result.valid() || (future_status != std::future_status::ready)) {
            this->sendLogWarning(
                "Failed to receive confirmation from the FleetInfo server (target)!"
            );
            this->fleetinfo_client_->prune_pending_requests();
            return;
        }
    }

    this->setAndNotifyRendezvousHandled();
    this->initiateOffboardMode();
}

// This is only executed by non-leaders; FleetInfo response handling - Target position
void Pelican::processLeaderResponse(rclcpp::Client<comms::srv::FleetInfo>::SharedFuture future) {
    // Wait for the specified amount or until the result is available
    this->sendLogDebug("Getting response...");
    auto status = future.wait_for(std::chrono::seconds(constants::SERVICE_FUTURE_WAIT_SECS));

    if (status != std::future_status::ready) {
        this->sendLogDebug("Service not ready yet...");
        return;
    }

    auto response = future.get();
    this->sendLogInfo("Target received: {}", response->target);

    this->commenceSetActualTargetHeight(response->target.z);
    this->initiateSetTargetPosition(response->target);
}

/************************* Command-related *************************/
bool Pelican::checkCommandMsgValidity(const comms::msg::Command msg) {
    // Valid agent ID
    if (msg.agent <= 0) {
        this->sendLogWarning("Received command from invalid agent ID ({})", msg.agent);
        return false;
    }

    // Exclude handling of my messages
    if (((msg.leader_id == this->getID()) && !msg.ack) || (msg.agent == this->getID())) {
        this->sendLogDebug("Intercepted my own command{}", msg.apply ? " execution" : "");
        return false;
    };

    // Valid term ids
    if (msg.term_id < this->getCurrentTerm()) {
        this->sendLogWarning("Received old command");
        return false;
    }
    if ((msg.prev_term != msg.term_id - 1) || (msg.prev_term != this->getCurrentTerm() - 1)) {
        this->sendLogWarning(
            "Invalid command message received! prev_term:{} term_id:{}", msg.prev_term, msg.term_id
        );
        return false;
    }
    if (msg.term_id > this->getCurrentTerm()) {
        this->sendLogWarning(
            "Aligning term ID with the one received in a command ({})", msg.term_id
        );
        this->setTerm(msg.term_id);
    }

    // Valid command
    if (msg.command <= NO_COMMAND) {
        this->sendLogWarning("Invalid command received! {}", commands_to_string(msg.command));
        return false;
    }

    // Index in AppendEntryRPC vector
    this->rpcs_mutex_.lock();
    unsigned int last_rpc_store = 0;
    unsigned int sec2last = 0;
    auto rpc_size = this->rpcs_vector_.size();
    unsigned int expected_index = rpc_size;
    if (rpc_size > 0)
        last_rpc_store = std::get<0>(this->rpcs_vector_.back());
    if (rpc_size > 2)
        sec2last = std::get<0>(this->rpcs_vector_.rbegin()[1]);
    this->rpcs_mutex_.unlock();

    // Valid previous command
    if (msg.apply) {
        if ((sec2last != 0) && (msg.prev_command != sec2last)) {
            this->sendLogWarning(
                "The last command is not coherent-> msg:{} expected(sec2last):{}", msg.prev_command,
                sec2last
            );
            return false;
        }
        // If only one RPC is in the vector, it'll be the one in scope

    } else {
        if ((last_rpc_store != 0) && (msg.prev_command != last_rpc_store)) {
            this->sendLogWarning(
                "The last command is not coherent-> msg:{} expected(last):{}", msg.prev_command,
                last_rpc_store
            );
            return false;
        }
    }

    // Valid index
    if (msg.index != expected_index) {
        this->sendLogWarning(
            "Received command with index {} greater than the number of stored RPCs (expected:{})",
            msg.index, expected_index
        );
    };

    return true;
}

bool Pelican::broadcastCommand(uint16_t command) {
    // Send command to other agents
    unsigned int attempt = 0;
    bool command_successful = false;
    while ((!command_successful) && (attempt < constants::MAX_BROADCAST_RETRIES)) {
        this->dispatch_mutex_.lock();
        this->dispatch_vector_.clear();
        this->dispatch_mutex_.unlock();

        this->sendLogInfo("Sending command {}", commands_to_string(command));
        this->sendAppendEntryRPC(this->getID(), command);
        command_successful = this->waitForRPCsAcks(command);

        attempt++;
    }
    if (attempt >= constants::MAX_BROADCAST_RETRIES) {
        this->sendLogWarning("Max retries reached while waiting for acks");
        return false;
    } else {
        this->sendLogDebug("Successfully sent command {}", commands_to_string(command));
    }

    // If enough acks have been received, then notify agents to execute the received command
    attempt = 0;
    bool ack_successful = false;
    while ((!ack_successful) && (attempt < constants::MAX_GET_ACK_RETRIES)) {
        this->dispatch_mutex_.lock();
        this->dispatch_vector_.clear();
        this->dispatch_mutex_.unlock();

        this->sendLogInfo(
            "Sending confirmation to execute command {}", commands_to_string(command)
        );
        this->sendAppendEntryRPC(this->getID(), command, false, true);
        ack_successful = this->waitForRPCsAcks(command, true);

        attempt++;
    }
    if (attempt >= constants::MAX_GET_ACK_RETRIES) {
        this->sendLogWarning("Max retries reached while waiting for command application's acks");
        return false;
    } else {
        this->appendEntry(command, this->getCurrentTerm());
        this->sendLogDebug(
            "Successfully sent confirmation to execute command {}", commands_to_string(command)
        );
        this->sendLogInfo("Command {} broadcasted", commands_to_string(command));
        return true;
    }
}

void Pelican::handleCommandDispatch(uint16_t command) {
    if (this->broadcastCommand(command) && this->executeRPCCommand(command)) {
        this->sendLogInfo("Fleet is executing command {}", commands_to_string(command));
        switch (command) {
            case TAKEOFF_COMMAND:
                this->setFlyingStatus();
                break;
            case LANDING_COMMAND:
                this->unsetFlyingStatus();
                break;
            case RENDEZVOUS:
                this->setCarryingStatus();
                break;
            default:
                this->sendLogDebug("Handling operations for command {} unknown", command);
        };
        this->setLastCmdStatus();
    } else {
        this->sendLogWarning("Fleet execution of command {} failed", commands_to_string(command));
        this->unsetLastCmdStatus();
    }
}

void Pelican::handleCommandReception(const comms::msg::Command msg) {
    // Do not even consider handling acks if I'm not the leader
    if (!this->isLeader() && msg.ack) {
        return;
    }

    this->sendLogDebug(
        "Handling command {}{}{} received from agent {} for term {} (prev_cmd:{}, prev_term:{}, "
        "index:{})",
        commands_to_string(msg.command), msg.apply ? " execution" : "", msg.ack ? " ack" : "",
        msg.agent, msg.term_id, msg.prev_command, msg.prev_term, msg.index
    );

    if (!this->checkCommandMsgValidity(msg))
        return;

    // If this is reached, the command message can be considered
    if (this->role_ != leader) {
        // Ignore ack msgs, which should come from other followers
        if (!msg.ack) {
            if (this->role_ == candidate) {
                this->sendLogWarning("Received a command from an external leader!");
                this->commenceSetElectionStatus(msg.leader_id);
                this->becomeFollower();
                return;
            }

            this->sendRPCAck(msg.leader_id, msg.command, msg.apply);

            if (!msg.apply) {
                this->appendEntry(msg.command, msg.term_id);
            } else {
                this->executeRPCCommand(msg.command);
            }
        }

    } else { // For the leader agent

        // TODO: check if I'm a leader and I receive a command from another leader:
        // if previous term, reject (and let them know?)
        // if next/equal term, step down if last command is not committed

        // The leader should only consider ack messages
        if (msg.ack) {
            // Send feedback to Action client
            auto feedback = std::make_shared<comms::action::TeleopData::Feedback>();
            this->dispatch_mutex_.lock();
            this->dispatch_vector_.push_back(msg);
            auto s = this->dispatch_vector_.size();
            this->dispatch_mutex_.unlock();
            feedback->agents_involved = msg.ack ? s : s + 1;
            feedback->last_joined = msg.agent;
            feedback->command = msg.command;
            feedback->execution = msg.apply;

            this->last_goal_mutex_.lock();
            this->last_goal_handle_->publish_feedback(feedback);
            this->last_goal_mutex_.unlock();
        }
    }
}

void Pelican::sendAppendEntryRPC(unsigned int leader, uint16_t command, bool ack, bool apply) {
    auto cmd_msg = comms::msg::Command();
    cmd_msg.agent = this->getID();
    cmd_msg.leader_id = leader;
    cmd_msg.term_id = this->getCurrentTerm();
    cmd_msg.command = command;
    // This could also be taken from the last stored command
    cmd_msg.prev_term = cmd_msg.term_id - 1;
    cmd_msg.ack = ack;
    cmd_msg.apply = apply;
    cmd_msg.prev_command = 0; // default init

    this->rpcs_mutex_.lock();
    if (!this->rpcs_vector_.empty())
        cmd_msg.prev_command = std::get<0>(this->rpcs_vector_.back());
    cmd_msg.index = this->rpcs_vector_.size();
    this->rpcs_mutex_.unlock();

    this->pub_to_dispatch_->publish(cmd_msg);
}

void Pelican::sendRPCAck(unsigned int leader, uint16_t command, bool apply) {
    this->sendLogInfo(
        "Sending ack for command {}{} to the Leader", commands_to_string(command),
        apply ? " execution" : ""
    );
    this->sendAppendEntryRPC(leader, command, true, apply);
}

bool Pelican::waitForRPCsAcks(uint16_t command, bool apply) {
    std::this_thread::sleep_for(this->rpcs_ack_timeout_);

    this->dispatch_mutex_.lock();
    auto app_vector = this->dispatch_vector_;
    this->dispatch_mutex_.unlock();

    unsigned int ack_received = std::count_if(
        app_vector.cbegin(), app_vector.cend(),
        [this, command, apply](const comms::msg::Command elem) {
            return (
                (elem.leader_id == this->getID()) && (elem.term_id == this->getCurrentTerm()) &&
                (elem.command == command) && (elem.ack) && (elem.apply == apply)
            );
        }
    );
    this->sendLogDebug(
        "Counted {} acks for command {}{}", ack_received, commands_to_string(command),
        apply ? " execution" : ""
    );

    // A log entry is committed once the leader that created the entry has
    // replicated it on a majority of the servers
    if (ack_received >= this->getNetworkSize() / 2) {
        this->sendLogInfo(
            "Enough acks received for command {}{}", commands_to_string(command),
            apply ? " execution" : ""
        );
        return true;
    }
    this->sendLogDebug(
        "Not enough acks received for command {}{}", commands_to_string(command),
        apply ? " execution" : ""
    );
    return false;
}

void Pelican::appendEntry(uint16_t command, unsigned int term) {
    this->sendLogInfo(
        "Appending entry for command {} for term {}", commands_to_string(command), term
    );
    std::lock_guard lock(this->rpcs_mutex_);
    this->rpcs_vector_.push_back(std::tie(command, term));
}

bool Pelican::executeRPCCommand(uint16_t command) {
    switch (command) {
        case TAKEOFF_COMMAND:
            if (this->initiateSetHome() && this->initiateArm() && this->initiateTakeoff()) {
                this->sendLogInfo("Takeoff operations initiated");
                return true;
            } else {
                this->sendLogWarning("Takeoff operations failed! Try again");
                return false;
            }
            break;
        case LANDING_COMMAND:
            if (this->initiateReturnToLaunchPosition()) {
                this->sendLogInfo("Operations to complete a land at initial position initiated");
                return true;
            } else {
                this->sendLogWarning("Landing operations failed! Try again");
                return false;
            }
            break;
        case EMERGENCY_LANDING:
            if (this->initiateLand()) {
                this->sendLogInfo("Emergency landing operations initiated");
                return true;
            } else {
                this->sendLogWarning("Emergency landing operations failed! Try again");
                return false;
            }
            break;
        case RENDEZVOUS: {
            std::thread rend_thread(&Pelican::rendezvousFleet, this);
            rend_thread.detach();

            this->sendLogDebug("Waiting for the rendezvous to start...");
            std::unique_lock lock(this->rendez_tristate_mutex_);
            this->rendezvous_handled_cv_.wait(lock, [this] {
                return this->rendezvous_handled_ != TriState::Floating;
            });

            bool res = static_cast<bool>(this->rendezvous_handled_);
            this->sendLogInfo("Rendezvous operations {}initiated!", res ? "" : "could not be ");
            return res;
            break;
        }
        default:
            this->sendLogWarning("Command received is not supported!");
            return false;
    }
}

// Function to publish FormationDesired messages
void Pelican::sendDesiredFormationPositions(
    std::unordered_map<unsigned int, geometry_msgs::msg::Point> desired_positions
) {
    std::vector<unsigned int> ids = this->getCoptersIDs();

    auto msg = comms::msg::FormationDesired();
    for (auto& id : ids) {
        comms::msg::NetworkVertex v;
        v.set__agent_id(id).set__position(desired_positions.at(id));
        msg.des_positions.push_back(v);
    }

    this->pub_to_formation_->publish(msg);
}
