#include "datapad.hpp"

void Datapad::landingPage() {
    std::cout << std::endl << "Select functionality" << std::endl;
    int choice;
    while (true) {
        std::cout << "(1) Ensure leader is elected" << std::endl;
        std::cout << "(2) Initiate takeoff" << std::endl;
        std::cout << "(3) Send payload position" << std::endl;
        std::cout << "(4) Send dropoff position" << std::endl;
        std::cout << "(5) Initiate landing" << std::endl;
        std::cout << "(0) Exit" << std::endl;
        std::cout << " >>> ";

        std::cin >> choice;

        this->running_mutex_.lock();
        auto ret = this->running_;
        this->running_mutex_.unlock();
        if (!ret)
            break;

        if (std::cin.fail()) {
            this->sendLogDebug("Wrong input!");
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Input is not a number!" << std::endl << std::endl;
        } else {
            this->sendLogDebug("Choice made: {}", choice);
            switch (choice) {
                case 0:
                    rclcpp::shutdown();
                    return;
                case 1:
                    this->contactLeader();
                    break;
                case 2:
                    this->unitSortie();
                    break;
                case 3:
                    break;
                case 4:
                    break;
                case 5:
                    this->backToLZ();
                    break;
                default:
                    this->sendLogWarning("Choice not supported!");
            }
            // TODO: dustoff/medevac for fault handling?
        }

        // Simulate some wait between loop iterations
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    };

    std::cout << "Ending..." << std::endl;
    this->sendLogDebug("Ending main functionality...");
}

void Datapad::contactLeader() {
    this->sendFleetInfo(true, false, false); // TODO: constant for readability?
}

void Datapad::processContact(rclcpp::Client<comms::srv::FleetInfoExchange>::SharedFuture future) {
    // Wait for 1s or until the result is available
    this->sendLogDebug("Getting response...");
    auto status = future.wait_for(std::chrono::seconds(1));

    if (status == std::future_status::ready) {
        auto response = future.get();

        if ((response->leader_id != this->leader_) && (this->leader_present_)) {
            this->sendLogWarning(
                "Leader must have changed without proper notifications! New leader: {}",
                response->leader_id
            );
            this->leader_ = response->leader_id;
        }

        if ((response->present) && (response->leader_id > 0)) {
            this->sendLogInfo("Agent {} is currently the fleet leader", response->leader_id);
            this->leader_present_ = true;
            this->leader_ = response->leader_id;
        }

        if (response->taking_off) {
            if (response->success) {
                this->sendLogInfo("Takeoff acknowledged");
                this->fleet_fying_ = true;
            } else {
                this->sendLogWarning("Takeoff not successful");
            }
        }

        if (response->landing) {
            if (response->success) {
                this->sendLogInfo("Land command acknowledged");
                this->fleet_fying_ = false;
            } else {
                this->sendLogWarning("Landing not successful");
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

    this->sendFleetInfo(false, true, false); // TODO: constant for readability?
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

    this->sendFleetInfo(false, false, true); // TODO: constant for readability?
}

void Datapad::sendFleetInfo(bool presence, bool takeoff, bool landing) {
    auto request = std::make_shared<comms::srv::FleetInfoExchange::Request>();
    request->presence = presence;
    request->takeoff = takeoff;
    request->landing = landing;

    // Search for a second, then log and search again
    int total_search_time = 0;
    while (!this->fleetinfo_client_->wait_for_service(std::chrono::seconds(SEARCH_LEADER_STEP)) &&
           total_search_time < MAX_SEARCH_TIME) {
        if (!rclcpp::ok()) {
            this->sendLogError("Client interrupted while waiting for service. Terminating...");
            return; // CHECK: do something else?
        }

        this->sendLogDebug("Service not available; waiting some more...");
        total_search_time += SEARCH_LEADER_STEP;
    };

    if (total_search_time < MAX_SEARCH_TIME) {
        this->sendLogDebug("Server available");

        // Send request
        auto result = this->fleetinfo_client_->async_send_request(
            request, std::bind(&Datapad::processContact, this, std::placeholders::_1)
        );
    } else {
        this->sendLogWarning("The server seems to be down. Please try again.");
    }
}
