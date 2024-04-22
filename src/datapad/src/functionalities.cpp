#include "datapad.hpp"

void Datapad::landingPage() {
    std::cout << std::endl << "Select functionality" << std::endl;
    this->sendLogDebug("Select functionality");

    std::string str_choice;
    int choice;
    bool input_error = false;
    while (true) {
        std::cout << "(1) Ensure leader is elected" << std::endl;
        std::cout << "(2) Initiate takeoff" << std::endl;
        std::cout << "(3) Send payload position" << std::endl;
        std::cout << "(4) Send dropoff position" << std::endl;
        std::cout << "(5) Initiate landing" << std::endl;
        std::cout << "(0) Exit" << std::endl;
        std::cout << " >>> ";
        this->sendLogDebug("(1) Ensure leader is elected");
        this->sendLogDebug("(2) Initiate takeoff");
        this->sendLogDebug("(3) Send payload position");
        this->sendLogDebug("(4) Send dropoff position");
        this->sendLogDebug("(5) Initiate landing");
        this->sendLogDebug("(0) Exit");

        std::cin >> str_choice;

        auto ret = this->isRunning();
        if (!ret)
            break;

        input_error = false;
        try {
            size_t pos;
            choice = std::stoi(str_choice, &pos);
            // Was the full string used to build the int?
            if (pos != str_choice.size())
                input_error = true;
        } catch (std::logic_error&) {
            input_error = true;
        }

        if (input_error) {
            std::cout << "Invalid choice given!" << std::endl << std::endl;
            this->sendLogDebug("Wrong input!");
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        } else {
            this->sendLogDebug("Choice made: {}", choice);
            switch (choice) {
                case 0:
                    this->sendLogDebug("Initiating 'shutdown' functionality");
                    rclcpp::shutdown();
                    return;
                case 1:
                    this->sendLogDebug("Initiating 'contactLeader' functionality");
                    this->contactLeader();
                    break;
                case 2:
                    this->sendLogDebug("Initiating 'unitSortie' functionality");
                    this->unitSortie();
                    break;
                case 3:
                    this->sendLogDebug("Initiating 'payloadExtraction' functionality");
                    this->payloadExtraction();
                    break;
                case 4:
                    this->sendLogDebug("Initiating 'payloadDropoff' functionality");
                    this->payloadDropoff();
                    break;
                case 5:
                    this->sendLogDebug("Initiating 'backToLZ' functionality");
                    this->backToLZ();
                    break;
                default:
                    this->sendLogWarning("Choice not supported!");
            }
            // TODO: dustoff/medevac for fault handling?
        }

        // Simulate some wait between loop iterations
        std::this_thread::sleep_for(std::chrono::milliseconds(constants::HOMEPAGE_LOOP_WAIT_SECS));
    };

    std::cout << "Ending..." << std::endl;
    this->sendLogDebug("Ending main functionality...");
}

void Datapad::contactLeader() {
    this->sendLogDebug("Contacting leader...");
    this->teleopTaskServer(Flags().SetPresence());
}

void Datapad::processFleetLeaderCommandAck(
    rclcpp::Client<comms::srv::TeleopData>::SharedFuture future
) {
    // Wait for the specified amount or until the result is available
    this->sendLogDebug("Getting response...");
    auto status = future.wait_for(std::chrono::seconds(constants::SERVICE_FUTURE_WAIT_SECS));

    if (status == std::future_status::ready) {
        auto response = future.get();

        // Verify if there's a new leader in the fleet
        if ((response->leader_id != this->leader_) && (this->leader_present_)) {
            this->sendLogWarning(
                "Leader must have changed without proper notifications! New leader: {}",
                response->leader_id
            );
            this->leader_ = response->leader_id;
        }

        // Leader notification
        if ((response->present) && (response->leader_id > 0)) {
            this->sendLogInfo("Agent {} is currently the fleet leader", response->leader_id);
            this->leader_present_ = true;
            this->leader_ = response->leader_id;
        }

        // Fleet taking off
        if (response->taking_off) {
            if (response->success) {
                this->sendLogInfo("Takeoff acknowledged");
                this->fleet_fying_ = true;
            } else {
                this->sendLogWarning("Takeoff operations not successful");
            }
        }

        // Fleet landing
        if (response->landing) {
            if (response->success) {
                this->sendLogInfo("Land command acknowledged");
                this->fleet_fying_ = false;
            } else {
                this->sendLogWarning("Landing operations not successful");
            }
        }

        // Sending payload initial position
        if (response->retrieval) {
            if (response->success) {
                this->sendLogInfo("Payload position acquired");
                this->transport_wip_ = true;
            } else {
                this->sendLogWarning("Could not transmit payload position");
            }
        }

        // Sending payload desired position
        if (response->dropoff) {
            if (response->success) {
                this->sendLogInfo("Dropoff position acquired");
                this->transport_wip_ = false;
            } else {
                this->sendLogWarning("Could not transmit dropoff position");
            }
        }

    } else {
        this->sendLogDebug("Service not ready yet...");
    }
}

void Datapad::unitSortie() {
    // Check if the fleet has a leader, since everything goes through it
    if (!this->leader_present_) {
        this->sendLogWarning(
            "No leader has been detected in the fleet! Please make sure it is present"
        );
        return;
    }

    // Check that the fleet is indeed flying
    if (this->fleet_fying_) {
        this->sendLogWarning("Units are already deployed!");
        return;
    }

    this->teleopTaskServer(Flags().SetTakeoff());
}

void Datapad::backToLZ() {
    // Check if the fleet has a leader, since everything goes through it
    if (!this->leader_present_) {
        this->sendLogWarning(
            "No leader has been detected in the fleet! Please make sure it is present"
        );
        return;
    }
    // Check that the fleet is indeed flying
    if (!this->fleet_fying_) {
        this->sendLogWarning("Units are not deployed!");
        return;
    }

    this->teleopTaskServer(Flags().SetLanding());
}

void Datapad::payloadExtraction() {
    // Check if the fleet has a leader, since everything goes through it
    if (!this->leader_present_) {
        this->sendLogWarning(
            "No leader has been detected in the fleet! Please make sure it is present"
        );
        return;
    }
    // Check that the fleet is not already busy
    if (this->transport_wip_) {
        this->sendLogWarning("Payload is already engaged!");
        return;
    }

    this->teleopTaskServer(Flags().SetRetrieval());
}

void Datapad::payloadDropoff() {
    // Check if the fleet has a leader, since everything goes through it
    if (!this->leader_present_) {
        this->sendLogWarning(
            "No leader has been detected in the fleet! Please make sure it is present"
        );
        return;
    }
    // Check that the payload is engaged
    if (!this->transport_wip_) {
        this->sendLogWarning("The fleet is not carrying anything!");
        return;
    }

    this->teleopTaskServer(Flags().SetDropoff());
}

void Datapad::teleopTaskServer(Flags flags) { // Server side of TeleopData service
    auto request = std::make_shared<comms::srv::TeleopData::Request>();
    request->presence = flags.GetPresence();
    request->takeoff = flags.GetTakeoff();
    request->landing = flags.GetLanding();
    request->retrieval = flags.GetRetrieval();
    request->dropoff = flags.GetDropoff();

    float coordinates[3];
    char coord_names[] = {'x', 'y', 'z'};
    // Add position if needed
    if (flags.GetRetrieval() || flags.GetDropoff()) {
        this->sendLogDebug("Enter desired coordinates: ");
        std::cout << "Enter desired coordinates" << std::endl;
        int i = 0;
        while (i < 3) {
            std::cout << coord_names[i] << ": " << std::endl;
            std::cin >> coordinates[i];
            if (std::cin.fail()) {
                std::cout << "Wrong input!" << std::endl << std::endl;
                this->sendLogDebug("Wrong input!");
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            } else {
                this->sendLogDebug("input: {}", coordinates[i]);
                i++;
            }
        };

        request->x = coordinates[0];
        request->y = coordinates[1];
        request->z = coordinates[2];
    }

    // Search for a second, then log and search again
    unsigned int total_search_time = 0;
    while (!this->teleopdata_client_->wait_for_service(
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
        this->sendLogDebug("Server available");

        // Send request
        auto result = this->teleopdata_client_->async_send_request(
            request, std::bind(&Datapad::processFleetLeaderCommandAck, this, std::placeholders::_1)
        );

        // If the callback is never called, because we never got a reply for the service server,
        // clean up pending requests to use less memory
        auto future_status =
            result.wait_for(std::chrono::seconds(constants::SERVICE_FUTURE_WAIT_SECS));
        if (!result.valid() || (future_status != std::future_status::ready)) {
            this->sendLogWarning("Async request to the fleet leader could not be completed!");
            this->teleopdata_client_->prune_pending_requests();
            return;
        }

    } else {
        this->sendLogWarning("The server seems to be down. Please try again.");
    }
}
