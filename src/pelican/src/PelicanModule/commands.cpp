#include "PelicanModule/pelican.hpp"

void Pelican::rogerWillCo(
    const std::shared_ptr<comms::srv::FleetInfoExchange::Request> request,
    const std::shared_ptr<comms::srv::FleetInfoExchange::Response> response
) {
    response->leader_id = this->id_;

    if (request->presence) {
        this->sendLogDebug("Notifying I'm the leader to the user!");
        response->present = true;
        response->taking_off = false;
        response->landing = false;
    }

    if (request->takeoff) {
        response->present = false;
        response->landing = false;
        if (!this->flying_) { // CHECK condition
            this->sendLogDebug("Notifying start of takeoff!");
            response->taking_off = true;

            std::thread takeoff_thread(&Pelican::broadcastCommand, this, TAKEOFF_COMMAND);
            takeoff_thread.detach();
        } else {
            this->sendLogWarning("Duplicate 'takeoff' command received!");
        }
    }

    if (request->landing) {
        response->present = false;
        response->taking_off = false;
        if (this->flying_) { // CHECK condition
            this->sendLogDebug("Notifying start of landing operations!");
            response->landing = true;

            std::thread landing_thread(&Pelican::broadcastCommand, this, LANDING_COMMAND);
            landing_thread.detach();
        } else {
            this->sendLogWarning("Duplicate 'land' command received!");
        }
    }
}

void Pelican::broadcastCommand(unsigned int command) {
    this->appendEntry(command, this->current_term_);

    // Theoretically, the leader keeps trying to send entries indefinitely in Raft
    // Here we try for a max amount of times
    unsigned int attempt = 0;
    bool command_successful = false;
    while ((!command_successful) && (attempt < MAX_RETRIES)) {
        this->dispatch_mutex_.lock();
        this->dispatch_vector_.clear();
        this->dispatch_mutex_.unlock();

        this->sendLogInfo("Sending command {}", command);
        this->sendAppendEntryRPC(this->id_, command);
        command_successful = this->waitForAcks(command);

        attempt++;
    }
    if (attempt >= MAX_RETRIES) {
        this->sendLogWarning("Max retries reached while waiting for acks");
        return;
    } else
        this->sendLogDebug("Successfully sent command {}", command);

    // If enough acks have been received, then notify agents to execute the received command
    attempt = 0;
    bool ack_successful = false;
    while ((!ack_successful) && (attempt < MAX_RETRIES)) {
        this->dispatch_mutex_.lock();
        this->dispatch_vector_.clear();
        this->dispatch_mutex_.unlock();

        this->sendLogInfo("Sending confirmation to execute command {}", command);
        this->sendAppendEntryRPC(this->id_, command, true);
        ack_successful = this->waitForAcks(command, true);

        attempt++;
    }
    if (attempt >= MAX_RETRIES) {
        this->sendLogWarning("Max retries reached while waiting for command application's acks");
    } else
        this->sendLogDebug("Successfully sent confirmation to execute command {}", command);
    // this->commenceTakeoff();
    this->sendLogInfo("Fleet takeoff initiated");
}

bool Pelican::waitForAcks(unsigned int command, bool apply) {
    rclcpp::Time start = this->now();
    std::this_thread::sleep_for(this->ack_timeout_);

    this->dispatch_mutex_.lock();
    auto app_vector = this->dispatch_vector_;
    this->dispatch_mutex_.unlock();

    int ack_received = std::count_if(
        app_vector.cbegin(), app_vector.cend(),
        [this, command, apply](const comms::msg::Command elem) {
            return (
                (elem.leader_id == this->id_) && (elem.term_id == this->current_term_) &&
                (elem.command == command) && (elem.ack) && (elem.apply == apply)
            );
        }
    );
    this->sendLogDebug("Counted {} acks for command {}", ack_received, command);

    // A log entry is committed once the leader that created the entry has
    // replicated it on a majority of the servers
    if (ack_received >= this->network_size_ / 2 + 1) {
        this->sendLogInfo("Enough acks received for command {} (apply={})", command, apply);
        return true;
    }
    this->sendLogDebug("Not enough acks received for command {}", command);
    return false;
}

void Pelican::sendAck(unsigned int leader, unsigned int command, bool apply) {
    this->sendLogInfo("Sending ack for command {} (apply={})", command, apply);
    this->sendAppendEntryRPC(leader, command, true, apply);
}

void Pelican::sendAppendEntryRPC(unsigned int leader, unsigned int command, bool ack, bool apply) {
    auto cmd_msg = comms::msg::Command();
    cmd_msg.agent = this->id_;             // CHECK: mutex?
    cmd_msg.leader_id = leader;
    cmd_msg.term_id = this->current_term_; // CHECK: mutex?
    cmd_msg.command = command;
    cmd_msg.ack = ack;
    cmd_msg.apply = apply;
    // TODO: index

    this->pub_to_dispatch_->publish(cmd_msg);
}

void Pelican::handleCommand(const comms::msg::Command msg) {
    if (msg.agent <= 0) { // Valid agent ID
        this->sendLogWarning("Received command from invalid agent ID");
        return;
    }
    if ((msg.leader_id == this->id_) && !msg.ack) { // Exclude case of leader-me sending messages
        this->sendLogDebug("Intercepted my own command");
        return;                                     // simply ignore
    };
    if (msg.term_id < this->current_term_) {
        this->sendLogWarning("Received old command (referring to term {})", msg.term_id);
        return;
    }
    if ((msg.prev_term != msg.term_id - 1) || (msg.prev_term != this->current_term_)) {
        this->sendLogWarning("Invalid command message received!");
        return;
    }
    if (msg.command <= NO_COMMAND) {
        this->sendLogWarning("Invalid command received!");
        return;
    }
    this->entries_mutex_.lock();
    unsigned int size = this->entries_rpcs_.size();
    this->entries_mutex_.unlock();
    if (msg.index >= size + 1) {
        /* request Logs if follower*/
    };
    if (msg.term_id > this->current_term_) {
        this->sendLogWarning(
            "Aligning term ID (current: {}) with the one received from command ({})",
            this->current_term_, msg.term_id
        );
        this->setTerm(msg.term_id);
    }

    // TODO: check if I'm a leader and I receive a command from another leader:
    // if previous term, reject (and let them know?)
    // if next/equal term, step down if last command is not committed

    // If this is reached, the command message can be considered
    if (this->role_ != leader) {
        if (!msg.apply && !msg.ack) { // Ignore acks from non-leader agents
            this->appendEntry(msg.command, msg.term_id);

            if (this->role_ == candidate) {
                this->sendLogWarning("Received a command from an external leader!");
                this->becomeFollower();
            }

        } else {
            /*execute command; TODO */
        }

        if (!msg.ack)
            this->sendAck(msg.leader_id, msg.command, msg.apply);

    } else { // For the leader
        // The leader should only consider ack messages
        if (msg.ack) {
            this->dispatch_mutex_.lock();
            this->dispatch_vector_.push_back(msg);
            this->dispatch_mutex_.unlock();
        }
    }
}

void Pelican::appendEntry(unsigned int command, unsigned int term) {
    this->sendLogInfo("Appending entry for command {} in term {}", command, term);
    std::lock_guard<std::mutex> lock(this->entries_mutex_);
    this->entries_rpcs_.push_back(std::tie(command, term));
}
