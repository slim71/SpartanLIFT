#include "PelicanModule/pelican.hpp"
#include "UNSCModule/unsc.hpp"

void UNSCModule::arm() {
    this->signalPublishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        px4_msgs::msg::VehicleCommand::ARMING_ACTION_ARM
    );

    this->sendLogInfo("Arm command sent");
}

void UNSCModule::disarm() {
    this->signalPublishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        px4_msgs::msg::VehicleCommand::ARMING_ACTION_DISARM
    );

    this->sendLogInfo("Disarm command sent");
}

// Pitch| Empty| Empty| Yaw| Latitude| Longitude| Altitude|
void UNSCModule::takeoff(unsigned int height) {
    if (height > 0) {
        this->signalPublishVehicleCommand(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, NAN, NAN, NAN, NAN, NAN, NAN,
            height
        );
    } else {
        this->signalPublishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF);
    }

    if (waitForAck(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF)) {
        this->arm();
    }
}

// Empty| Empty| Empty| Yaw| Latitude| Longitude| Altitude|
void UNSCModule::land() {
    this->signalPublishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
}

// Use current?| Empty| Empty| Empty| Latitude| Longitude| Altitude|
void UNSCModule::setHome() {
    this->signalPublishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_HOME,
        1.0 // TODO: find or define constant
    );

    this->sendLogInfo("Set home sent");
}

bool UNSCModule::waitForAck(uint16_t cmd) {
    auto start_time = this->gatherTime().nanoseconds() / 1000000; // [ms]
    this->sendLogDebug("ACK wait started at timestamp {}", start_time);

    // Check for a bit, in case the ACK has not been received yet
    while (this->gatherTime().nanoseconds() / 1000000 - start_time < 100) {
        std::optional<px4_msgs::msg::VehicleCommandAck> ack = this->gatherAck();

        if (ack) { // at least one ACK received
            this->sendLogDebug(
                "Stored ACK found! cmd:{} timestamp:{}", ack->command, ack->timestamp
            );
            if ((ack->command == cmd)) {
                auto result =
                    ack->result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
                return result;
            }
        }

        // Wait for a bit before retrying
        std::this_thread::sleep_for(std::chrono::microseconds(10000));
    }
    this->sendLogDebug("ACK wait finished");

    return false;
}

void UNSCModule::runPreChecks() {
    // Delete wall timer to have it set off only once
    cancelTimer(this->starting_timer_);

    this->signalPublishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_RUN_PREARM_CHECKS);

    auto start_time = this->gatherTime().nanoseconds() / 1000000; // [ms]
    this->sendLogDebug("Status data wait started at timestamp {}", start_time);

    // Check for a bit, in case the ACK has not been received yet
    while (this->gatherTime().nanoseconds() / 1000000 - start_time < 100) {
        std::optional<px4_msgs::msg::VehicleStatus> status = this->gatherStatus();

        if (status) { // at least one status received
            this->sendLogInfo(
                "Stored status message found! timestamp:{} arming_state:{} nav_state:{} "
                "pre_flight_checks_pass:{} failsafe:{}",
                status->timestamp, status->arming_state, status->nav_state,
                status->pre_flight_checks_pass, status->failsafe
            );

            // Check whether the agent in the simulation is good to go
            this->sitl_ready_ =
                status->pre_flight_checks_pass && !status->failsafe &&
                ((1u << status->nav_state) != 0) && // taken from the PX4 commander
                status->arming_state < px4_msgs::msg::VehicleStatus::ARMING_STATE_STANDBY_ERROR;

            if (this->sitl_ready_)
                this->sendLogInfo("Simulation ready!");
            else
                this->sendLogError("Errors in the simulation detected!");

            this->setHome();

            // TODO: delete
            this->starting_timer_ = this->node_->create_wall_timer(
                this->briefing_time_,
                [this] {
                    this->takeoff();
                },
                this->gatherReentrantGroup()
            );

            return;
        }

        // Wait for a bit before retrying
        std::this_thread::sleep_for(std::chrono::microseconds(10000));
    }

    this->sendLogDebug("Status data wait finished");

    // Default to the hypothesis that the simulation has some problem
    this->sitl_ready_ = false;
    this->sendLogError("Errors in the simulation detected!");
}

// Empty| Empty| Empty| Empty| Empty| Empty| Empty|
void UNSCModule::returnToLaunchPosition() {
    this->signalPublishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
    );

    this->sendLogInfo("ReturnToLaunch sent");
}

void UNSCModule::setAndMaintainOffboardMode(float x, float y, float z, float yaw) {
    if (this->offboard_setpoint_counter_ == 10) {
        // Change to Offboard mode after 10 setpoints
        this->sendLogDebug("Changing to Offboard mode!");
        this->signalPublishVehicleCommand(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6
        ); // CHECK: can use px4 constants? others?
    }

    // offboard_control_mode needs to be paired with trajectory_setpoint
    this->signalPublishOffboardControlMode();

    // stop the counter after reaching 11
    if (this->offboard_setpoint_counter_ < 11) {
        this->offboard_setpoint_counter_++;
    }
}

void UNSCModule::setPositionMode() {
    this->signalPublishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_HOME, 1, 0);

    this->sendLogInfo("Set position mode sent");
}
