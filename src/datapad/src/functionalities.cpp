#include "datapad.hpp"

void Datapad::landingPage() {
    std::cout << std::endl << "Select functionality" << std::endl;
    this->sendLogDebug("Select functionality");

    std::string str_choice;
    int choice;
    bool input_error = false;
    while (true) {
        std::cout << std::endl;
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

        // Handle user input
        std::cin >> str_choice;
        if (!this->isRunning())
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
                    if (!this->leader_present_) {
                        this->sendLogWarning("No leader has been detected in the fleet! Please "
                                             "make sure it is present");
                        break;
                    }
                    this->sendLogDebug("Initiating 'unitSortie' functionality");
                    this->unitSortie();
                    break;
                case 3:
                    if (!this->leader_present_) {
                        this->sendLogWarning("No leader has been detected in the fleet! Please "
                                             "make sure it is present");
                        break;
                    }
                    this->sendLogDebug("Initiating 'payloadExtraction' functionality");
                    this->payloadExtraction();
                    break;
                case 4:
                    if (!this->leader_present_) {
                        this->sendLogWarning("No leader has been detected in the fleet! Please "
                                             "make sure it is present");
                        break;
                    }
                    this->sendLogDebug("Initiating 'payloadDropoff' functionality");
                    this->payloadDropoff();
                    break;
                case 5:
                    if (!this->leader_present_) {
                        this->sendLogWarning("No leader has been detected in the fleet! Please "
                                             "make sure it is present");
                        break;
                    }
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
    this->teleopTaskClient(Flags().SetPresence());
}

// Process the final result of the TeleopData goal
void Datapad::processFleetLeaderCommandAck(
    const rclcpp_action::ClientGoalHandle<comms::action::TeleopData>::WrappedResult& result
) {
    this->sendLogDebug("Got response from the server");

    // Check if the server succeeded in handling the request
    bool success = false;
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            success = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            this->sendLogWarning("Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            this->sendLogWarning("Goal was canceled");
            break;
        default:
            this->sendLogWarning("Unknown result code, but still not succesful");
            break;
    }

    auto response = result.result;

    // Verify if there's a new leader in the fleet
    if ((response->leader_id != this->leader_) && (this->leader_present_)) {
        this->sendLogWarning(
            "Leader must have changed without proper notifications! New leader: {}",
            response->leader_id
        );
        this->leader_ = response->leader_id;
    }

    // Leader notification
    if ((response->presence) && (response->leader_id > 0)) {
        this->sendLogInfo("Agent {} is currently the fleet leader", response->leader_id);
        this->leader_present_ = true;
        this->leader_ = response->leader_id;
    }

    // Fleet taking off
    if (response->taking_off) {
        if (success) {
            this->sendLogInfo("Takeoff acknowledged");
            this->fleet_fying_ = true;
        } else {
            this->sendLogWarning("Takeoff operations not successful");
        }
    }

    // Fleet landing
    if (response->landing) {
        if (success) {
            this->sendLogInfo("Land command acknowledged");
            this->fleet_fying_ = false;
        } else {
            this->sendLogWarning("Landing operations not successful");
        }
    }

    // Sending payload initial position
    if (response->retrieval) {
        if (success) {
            this->sendLogInfo("Payload position acquired");
            this->transport_wip_ = true;
        } else {
            this->sendLogWarning("Could not transmit payload position");
        }
    }

    // Sending payload desired position
    if (response->dropoff) {
        if (success) {
            this->sendLogInfo("Dropoff position acquired");
            this->transport_wip_ = false;
        } else {
            this->sendLogWarning("Could not transmit dropoff position");
        }
    }
    // Only to have a clearer interface for the user
    std::cout << " >>> ";
}

void Datapad::unitSortie() {
    // Check that the fleet is indeed flying
    if (this->fleet_fying_) {
        this->sendLogWarning("Units are already deployed!");
        return;
    }

    this->teleopTaskClient(Flags().SetTakeoff());
}

void Datapad::backToLZ() {
    // Check that the fleet is indeed flying
    if (!this->fleet_fying_) {
        this->sendLogWarning("Units are not deployed!");
        return;
    }

    this->teleopTaskClient(Flags().SetLanding());
}

void Datapad::payloadExtraction() {
    // Check that the fleet is not already busy
    if (this->transport_wip_) {
        this->sendLogWarning("Payload is already engaged!");
        return;
    }

    this->teleopTaskClient(Flags().SetRetrieval());
}

void Datapad::payloadDropoff() {
    // Check that the payload is engaged
    if (!this->transport_wip_) {
        this->sendLogWarning("The fleet is not carrying anything!");
        return;
    }

    this->teleopTaskClient(Flags().SetDropoff());
}

// TeleopData - request generator
void Datapad::teleopTaskClient(Flags flags) {
    auto request = comms::action::TeleopData::Goal();
    request.presence = flags.GetPresence();
    request.takeoff = flags.GetTakeoff();
    request.landing = flags.GetLanding();
    request.retrieval = flags.GetRetrieval();
    request.dropoff = flags.GetDropoff();

    // Add position if needed
    if (flags.GetRetrieval()) {
        std::thread cargo_thread(&Datapad::askForCargoPoint, this);
        cargo_thread.detach();

        this->sendLogDebug("Waiting for the Cargo to send a position...");
        std::unique_lock lock(this->cargo_tristate_mutex_);
        this->cargo_cv_.wait(lock, [this] {
            return this->cargo_handled_ != TriState::Floating;
        });
        lock.unlock();
        if (this->isCargoHandled() != TriState::True) {
            this->sendLogWarning("Cargo did not share a valid position!");
            return;
        }
        this->sendLogDebug("Valid cargo position received");

        std::lock_guard odom_lock(this->cargo_mutex_);
        request.x = this->cargo_odom_.x;
        request.y = this->cargo_odom_.y;
        request.z = this->cargo_odom_.z + 3.0;

    } else if (flags.GetDropoff()) {
        float coordinates[2];
        char coord_names[] = {'x', 'y'};
        this->sendLogDebug("Enter desired coordinates: ");
        std::cout << "Enter desired coordinates" << std::endl;
        int i = 0;
        while (i < sizeof(coordinates) / sizeof(float)) {
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

        request.x = coordinates[0];
        request.y = coordinates[1];
        request.z = 0;
    }

    // Bind callbacks for feedback handling
    auto send_goal_options = rclcpp_action::Client<comms::action::TeleopData>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&Datapad::analyzeTeleopDataResponse, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(
        &Datapad::parseTeleopDataFeedback, this, std::placeholders::_1, std::placeholders::_2
    );
    send_goal_options.result_callback =
        std::bind(&Datapad::processFleetLeaderCommandAck, this, std::placeholders::_1);

    // Search for a second, then log and search again
    unsigned int total_search_time = 0;
    while (!this->teleopdata_client_->wait_for_action_server(
               std::chrono::seconds(constants::SEARCH_LEADER_STEP_SECS)
           ) &&
           total_search_time < constants::MAX_SEARCH_TIME_SECS) {
        if (!rclcpp::ok()) {
            this->sendLogError(
                "Client interrupted while waiting for TeleopData action service. Terminating..."
            );
            return;
        }

        this->sendLogDebug("TeleopData action service not available; waiting some more...");
        total_search_time += constants::SEARCH_LEADER_STEP_SECS;
    };

    if (total_search_time < constants::MAX_SEARCH_TIME_SECS) {
        this->sendLogDebug("TeleopData action server available");
        // Send request
        auto result = this->teleopdata_client_->async_send_goal(request, send_goal_options);
    } else {
        this->sendLogWarning("The TeleopData action server seems to be down. Please try again.");
    }
}

// CargoPoint - request generator
void Datapad::askForCargoPoint() {
    auto service_name = this->cargopoint_client_->get_service_name();
    // Search for a second, then log and search again if needed
    unsigned int total_search_time = 0;
    while (!this->cargopoint_client_->wait_for_service(
               std::chrono::seconds(constants::SEARCH_LEADER_STEP_SECS)
           ) &&
           total_search_time < constants::MAX_SEARCH_TIME_SECS) {
        if (!rclcpp::ok()) {
            this->sendLogError(
                "Client interrupted while waiting for {} service. Terminating...", service_name
            );
            this->unsetAndNotifyCargoHandled();
            return;
        }

        this->sendLogDebug("{} service not available; waiting some more...", service_name);
        total_search_time += constants::SEARCH_LEADER_STEP_SECS;
    };

    if (total_search_time >= constants::MAX_SEARCH_TIME_SECS) {
        this->sendLogWarning("The {} server seems to be down. Please try again.", service_name);
        this->unsetAndNotifyCargoHandled();
        return;
    }
    this->sendLogDebug("{} server available", service_name);

    // Send request
    auto request = std::make_shared<comms::srv::CargoPoint::Request>();
    auto async_request_result = this->cargopoint_client_->async_send_request(
        request, std::bind(&Datapad::storeCargoPoint, this, std::placeholders::_1)
    );
    // Check if request was accepted and cleanup if not (not to waste memory)
    auto future_status =
        async_request_result.wait_for(std::chrono::seconds(constants::SERVICE_FUTURE_WAIT_SECS));
    if (!async_request_result.valid() || (future_status != std::future_status::ready)) {
        this->sendLogWarning("Failed to receive confirmation from the {} server!", service_name);
        this->cargopoint_client_->prune_pending_requests();
        return;
    }
}
