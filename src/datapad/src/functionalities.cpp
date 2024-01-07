#include "datapad.hpp"

void Datapad::landingPage() {
    std::cout << std::endl << "Select functionality" << std::endl;
    int choice;
    while (true) {
        if (this->contact_client_)
            this->contact_client_.reset();

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
                    break;
                case 3:
                    break;
                case 4:
                    break;
                case 5:
                    break;
                default:
                    this->sendLogWarning("Choice not supported!");
            }
        }

        // Simulate some wait between loop iterations
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    };

    std::cout << "Ending..." << std::endl;
    this->sendLogDebug("Ending main functionality...");
}

void Datapad::contactLeader() {
    this->contact_client_ = this->create_client<comms::srv::Contact>("contactLeader_service");

    auto request = std::make_shared<comms::srv::Contact::Request>();
    request->question = true;

    // Search for a second, then log and search again
    int total_search_time = 0;
    while (!this->contact_client_->wait_for_service(std::chrono::seconds(SEARCH_LEADER_STEP)) &&
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
        auto result = this->contact_client_->async_send_request(
            request, std::bind(&Datapad::processContact, this, std::placeholders::_1)
        );
    } else {
        this->sendLogWarning("The server seems to be down. Please try again.");
    }
}

void Datapad::processContact(rclcpp::Client<comms::srv::Contact>::SharedFuture future) {
    // Wait for 1s or until the result is available
    this->sendLogDebug("Getting response...");
    auto status = future.wait_for(std::chrono::seconds(1));

    if (status == std::future_status::ready) {
        auto response = future.get();
        if (response->present) {
            this->sendLogInfo("Agent {} is currently the fleet leader", response->leader_id);
            this->leader_present_ = true;

        } else {
            this->sendLogInfo("The fleet has no leader yet");
            this->leader_present_ = false; // CHECK: redundant?
        }
    } else {
        this->sendLogDebug("Service not ready yet...");
    }
}
