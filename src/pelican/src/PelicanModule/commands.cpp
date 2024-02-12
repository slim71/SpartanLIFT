#include "PelicanModule/pelican.hpp"

void Pelican::rogerWillCo(
    const std::shared_ptr<comms::srv::FleetInfoExchange::Request> request,
    const std::shared_ptr<comms::srv::FleetInfoExchange::Response> response
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
            response->success = true;

            this->payload_position_ = {request->x, request->y, request->z};

            std::thread landing_thread(&Pelican::rendezvousFleet, this);
            landing_thread.detach();
        } else {
            this->sendLogWarning("The payload is already carrying the payload!");
        }
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
    this->last_command_mutex_.lock();
    unsigned int expected_cmd = this->last_command_stored_;
    unsigned int expected_index = this->last_occupied_index_;
    this->last_command_mutex_.unlock();

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
        command_successful = this->waitForAcks(command);

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
        ack_successful = this->waitForAcks(command, true);

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
    if (this->broadcastCommand(command) && this->executeCommand(command)) {
        this->sendLogInfo("Fleet is executing command {}", commands_to_string(command));
        switch (command) {
            case TAKEOFF_COMMAND:
                this->setFlyingStatus();
                break;
            case LANDING_COMMAND:
                this->unsetFlyingStatus();
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

            this->sendAck(msg.leader_id, msg.command, msg.apply);

            // Append Entry RPC
            if (!msg.apply) {
                this->appendEntry(msg.command, msg.term_id);

            } else { // Commit and execute command entry
                this->updateLastCommand();
                this->executeCommand(msg.command);
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
            this->updateLastCommand();
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

    this->last_command_mutex_.lock();
    cmd_msg.prev_command = this->last_command_stored_;
    cmd_msg.index = this->last_occupied_index_;
    this->last_command_mutex_.unlock();

    this->pub_to_dispatch_->publish(cmd_msg);
}

void Pelican::sendAck(unsigned int leader, uint16_t command, bool apply) {
    this->sendLogInfo(
        "Sending ack for command {}{} to the Leader", commands_to_string(command),
        apply ? " execution" : ""
    );
    this->sendAppendEntryRPC(leader, command, true, apply);
}

bool Pelican::waitForAcks(uint16_t command, bool apply) {
    rclcpp::Time start = this->now();
    std::this_thread::sleep_for(this->ack_timeout_);

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

bool Pelican::executeCommand(uint16_t command) {
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
        default:
            this->sendLogWarning("Command received is not supported!");
            return false;
    }
}

void Pelican::rendezvousFleet() {
    this->sendLogDebug(
        "Point to rendezvous to: ({:.4f}, {:.4f}, {:.4f})", this->payload_position_[0],
        this->payload_position_[1], this->payload_position_[2]
    );

    if (this->isLeader()) {
        this->initiateOffboardMode(
            this->payload_position_[0], this->payload_position_[1], this->payload_position_[2],
            0 // TODO: yaw?
        );
    }
}

void Pelican::updateLastCommand() {
    std::lock_guard<std::mutex> lock_cmd(this->last_command_mutex_);
    std::lock_guard<std::mutex> lock_rpc(this->rpcs_mutex_);
    this->last_command_stored_ = std::get<0>(this->rpcs_vector_.back());
    this->last_occupied_index_ = this->rpcs_vector_.size();
}
