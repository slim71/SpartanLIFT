#include "PelicanModule/pelican.hpp"

// TODO: make this work as a unique handling of whatever command, rendezvous and formation included
// This is only executed by the leader
void Pelican::rogerWillCo(
    const std::shared_ptr<comms::srv::TeleopData::Request> request,
    const std::shared_ptr<comms::srv::TeleopData::Response> response
) {
    // General all-purpose init
    response->leader_id = this->getID();
    response->present = false;
    response->taking_off = false;
    response->landing = false;
    response->retrieval = false;
    response->dropoff = false;
    response->success = false;

    // Handling leader ID request
    if (request->presence) {
        this->sendLogDebug("Notifying I'm the leader to the user!");
        response->present = true;
        response->success = true;
    }

    // Handling takeoff command
    if (request->takeoff) {
        response->taking_off = true;

        if (!this->isFlying()) {
            this->sendLogDebug("Notifying start of takeoff operations!");
            response->success = true;

            std::thread takeoff_thread(&Pelican::handleCommandDispatch, this, TAKEOFF_COMMAND);
            takeoff_thread.detach();
        } else {
            this->sendLogWarning("The fleet should already be flying!");
        }
    }

    // Handling landing command
    if (request->landing) {
        response->present = false;
        response->taking_off = false;
        response->landing = true;

        if (this->isFlying()) {
            this->sendLogDebug("Notifying start of landing operations!");
            response->success = true;

            std::thread landing_thread(&Pelican::handleCommandDispatch, this, LANDING_COMMAND);
            landing_thread.detach();
        } else {
            this->sendLogWarning("The fleet is not flying!");
        }
    }

    // Handling payload's initial position notification
    if (request->retrieval) {
        response->retrieval = true;

        if (!this->isCarrying()) {
            this->sendLogDebug("Notifying start of payload extraction operations!");
            this->sendLogInfo(
                "Payload is at ({:.4f},{:.4f},{:.4f})", request->x, request->y, request->z
            );
            response->success = true;

            this->setReferenceHeight(request->z);
            this->setTargetPosition(request->x, request->y, request->z);

            std::thread retrieval_thread(&Pelican::handleCommandDispatch, this, RENDEZVOUS);
            retrieval_thread.detach();
        } else {
            this->sendLogWarning("The payload is already carrying the payload!");
        }
    }
}

void Pelican::targetConvergence(
    const std::shared_ptr<comms::srv::FleetInfo::Request> request,
    const std::shared_ptr<comms::srv::FleetInfo::Response> response
) {
    auto pos = this->getTargetPosition();
    float target_height = this->getActualTargetHeight();
    if (pos) {
        std::vector<float> v = pos.value();
        response->target_x = v[0];
        response->target_y = v[1];
        response->target_z = target_height;

    } else { // CHECK: needed? the follower should be able to know what the target refers to
        response->formation = false;
        response->rendezvous = false;
    }
}

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
    this->last_rpc_command_mutex_.lock();
    unsigned int expected_cmd = this->last_rpc_command_stored_;
    unsigned int expected_index = this->last_occupied_index_;
    this->last_rpc_command_mutex_.unlock();

    // Valid previous command
    if (msg.prev_command != expected_cmd) {
        this->sendLogWarning(
            "The last command is not coherent-> msg:{} expected:{}", msg.prev_command, expected_cmd
        );
        return false;
    }

    // Valid index
    if (msg.index != expected_index) {
        this->sendLogWarning(
            "Received command with index {} greater than the number of stored RPCs (expected:{})",
            msg.index, expected_index
        );
        // TODO request Logs if follower
    };

    // TODO: check if I'm a leader and I receive a command from another leader:
    // if previous term, reject (and let them know?)
    // if next/equal term, step down if last command is not committed

    return true;
}

bool Pelican::broadcastCommand(uint16_t command) {
    this->appendEntry(command, this->getCurrentTerm());

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
    } else
        this->sendLogWarning("Fleet execution of command {} failed", commands_to_string(command));
}

void Pelican::handleCommandReception(const comms::msg::Command msg) {
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

            // Append Entry RPC
            if (!msg.apply) {
                this->appendEntry(msg.command, msg.term_id);

            } else { // Commit and execute command entry
                this->updateLastRPCCommandReceived();
                this->executeRPCCommand(msg.command);
            }
        }

    } else { // For the leader agent
        // The leader should only consider ack messages
        if (msg.ack) {
            this->dispatch_mutex_.lock();
            this->dispatch_vector_.push_back(msg);
            this->dispatch_mutex_.unlock();
        }
        if (msg.apply)
            this->updateLastRPCCommandReceived();
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

    this->last_rpc_command_mutex_.lock();
    cmd_msg.prev_command = this->last_rpc_command_stored_;
    cmd_msg.index = this->last_occupied_index_;
    this->last_rpc_command_mutex_.unlock();

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
    rclcpp::Time start = this->now();
    std::this_thread::sleep_for(this->rpcs_ack_timeout_);

    this->dispatch_mutex_.lock();
    auto app_vector = this->dispatch_vector_;
    this->dispatch_mutex_.unlock();

    int ack_received = std::count_if(
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
    if (ack_received >= this->getNetworkSize() / 2) { // +1 deleted: this node is already considered
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
    std::lock_guard<std::mutex> lock(this->rpcs_mutex_);
    this->rpcs_vector_.push_back(std::tie(command, term));
}

void Pelican::updateLastRPCCommandReceived() {
    std::lock_guard<std::mutex> lock_cmd(this->last_rpc_command_mutex_);
    std::lock_guard<std::mutex> lock_rpc(this->rpcs_mutex_);
    this->last_rpc_command_stored_ = std::get<0>(this->rpcs_vector_.back());
    this->last_occupied_index_ = this->rpcs_vector_.size();
}

bool Pelican::executeRPCCommand(uint16_t command) {
    // TODO: signal to datapad in case of operation failed
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
        case RENDEZVOUS:
            this->rendezvousFleet();

            // Give the mode switch some time to activate // CHECK wait value; maybe another way?
            std::this_thread::sleep_for(std::chrono::seconds(constants::RENDEZVOUS_WAIT_TIME_SECS));
            if (!this->initiateCheckOffboardEngagement()) {
                this->sendLogWarning("Rendezvous operations could not be accomplished!");
            } else {
                this->sendLogInfo(("Rendezvous initiated"));
                return true;
            }

            return false;
            break;
        default:
            this->sendLogWarning("Command received is not supported!");
            return false;
    }
}

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

        if (total_search_time < constants::MAX_SEARCH_TIME_SECS) {
            this->sendLogDebug("Rendezvous server available");

            auto request = std::make_shared<comms::srv::FleetInfo::Request>();
            // Send request
            auto async_request_result = this->fleetinfo_client_->async_send_request(
                request, std::bind(&Pelican::processLeaderResponse, this, std::placeholders::_1)
            );

            auto future_status = async_request_result.wait_for(
                std::chrono::seconds(constants::SERVICE_FUTURE_WAIT_SECS)
            );
            if (!async_request_result.valid() || (future_status != std::future_status::ready)) {
                this->sendLogWarning("Failed to receive a target position from the leader!");
                this->fleetinfo_client_->prune_pending_requests();
                return;
            }
        } else {
            this->sendLogWarning("The server seems to be down. Please try again.");
            return;
        }
    }

    this->initiateOffboardMode();
}

// This is only executed by non-leaders
void Pelican::processLeaderResponse(rclcpp::Client<comms::srv::FleetInfo>::SharedFuture future) {
    // Wait for the specified amount or until the result is available
    this->sendLogDebug("Getting response...");
    auto status = future.wait_for(std::chrono::seconds(constants::SERVICE_FUTURE_WAIT_SECS));

    if (status == std::future_status::ready) {
        auto response = future.get();

        unsigned int own_id = this->getID();
        int multiplier = own_id < this->initiateGetLeaderID() ? own_id : own_id - 1;
        float alpha = 2 * constants::PI / (this->getNetworkSize() - 1) * multiplier;
        float x = response->target_x + this->getROI() * cos(alpha);
        float y = response->target_y + this->getROI() * sin(alpha);
        this->sendLogDebug(
            "multiplier: {}, x: {:.4f}, y:{:.4f}, alpha: {:.4f}", multiplier, x, y, alpha
        );

        this->setReferenceHeight(response->target_z);
        this->setTargetPosition(x, y, response->target_z);
    } else {
        this->sendLogDebug("Service not ready yet...");
    }
}
