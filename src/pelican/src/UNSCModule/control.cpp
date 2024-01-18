#include "PelicanModule/pelican.hpp"
#include "UNSCModule/unsc.hpp"

bool UNSCModule::arm() {
    return this->sendToCommanderUnit(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        px4_msgs::msg::VehicleCommand::ARMING_ACTION_ARM
    );
}

bool UNSCModule::disarm() {
    return this->sendToCommanderUnit(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        px4_msgs::msg::VehicleCommand::ARMING_ACTION_DISARM
    );
}

// Pitch| Empty| Empty| Yaw| Latitude| Longitude| Altitude|
bool UNSCModule::takeoff(unsigned int height) {
    return this->sendToCommanderUnit(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, NAN, NAN, NAN, NAN, NAN, NAN,
        (height > 0) ? height : NAN
    );
}

// Empty| Empty| Empty| Yaw| Latitude| Longitude| Altitude|
bool UNSCModule::land() {
    return this->sendToCommanderUnit(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
}

// Use current?| Empty| Empty| Empty| Latitude| Longitude| Altitude|
bool UNSCModule::setHome() {
    return this->sendToCommanderUnit(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_HOME,
        1.0 // TODO: find or define constant
    );
}

// bool UNSCModule::setPositionMode() {
//     return this->sendToCommanderUnit(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_HOME, 1,
//     0);
// }

// Empty| Empty| Empty| Empty| Empty| Empty| Empty|
bool UNSCModule::returnToLaunchPosition() {
    return this->sendToCommanderUnit(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
    );
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

        // Wait for a bit before retrying CHECK: milli?
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
            this->sendLogDebug(
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

bool UNSCModule::sendToCommanderUnit(
    uint16_t command, float param1, float param2, float param3, float param4, float param5,
    float param6, float param7
) {
    unsigned int attempt = 1;
    do {
        this->sendLogDebug("Trying to send command {} and to get an ack", command);
        this->signalPublishVehicleCommand(
            command, param1, param2, param3, param4, param5, param6, param7
        );
        attempt++;
    } while ((!this->waitForAck(command)) && attempt < MAX_RETRIES);

    if (attempt >= MAX_RETRIES) {
        this->sendLogWarning("No ack received from the commander unit for command {}", command);
        return false;
    }

    this->sendLogInfo("Command {} sent", command);
    return true;
}
