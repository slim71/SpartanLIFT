/**
 * @file control.cpp
 * @author Simone Vollaro (slim71sv@gmail.com)
 * @brief Methods for communication with the PX4 commander.
 * @version 1.0.0
 * @date 2024-11-17
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "PelicanModule/pelican.hpp"
#include "UNSCModule/unsc.hpp"

/**
 * @brief Arms the vehicle by sending an arm command to the PX4 commander.
 *
 * @return true if the command is acknowledged; false otherwise.
 */
bool UNSCModule::arm() {
    return this->sendToCommanderUnit(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        px4_msgs::msg::VehicleCommand::ARMING_ACTION_ARM
    );
}

/**
 * @brief Disarms the vehicle by sending a disarm command to the PX4 commander.
 *
 * @return true if the command is acknowledged; false otherwise.
 */
bool UNSCModule::disarm() {
    return this->sendToCommanderUnit(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        px4_msgs::msg::VehicleCommand::ARMING_ACTION_DISARM
    );
}

/**
 * @brief Initiates a takeoff to a specified altitude.
 * Parameters for the general command: Pitch| Empty| Empty| Yaw| Latitude| Longitude| Altitude|
 *
 * @param height The target altitude for takeoff. If height is 0, the default behavior is applied.
 * @return true if the command is acknowledged; false otherwise.
 */
bool UNSCModule::takeoff(unsigned int height) {
    return this->sendToCommanderUnit(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, NAN, NAN, NAN, NAN, NAN, NAN,
        (height > 0) ? height : NAN
    );
}

/**
 * @brief Initiates a landing at the current position.
 * Parameters for the general command: Empty| Empty| Empty| Yaw| Latitude| Longitude| Altitude|
 *
 * @return true if the command is acknowledged; false otherwise.
 */
bool UNSCModule::land() {
    // Needed here because it could be called while the Offboard mode is active
    cancelTimer(this->offboard_timer_);
    return this->sendToCommanderUnit(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
}

/**
 * @brief Sets the current position as the home location for the vehicle.
 * Parameters for the general command: Use current?| Empty| Empty| Empty| Latitude| Longitude|
 * Altitude|
 *
 * @return true if the command is acknowledged; false otherwise.
 */
bool UNSCModule::setHome() {
    return this->sendToCommanderUnit(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_HOME, constants::CONFIRM_SET_HOME
    );
}

/**
 * @brief Commands the vehicle to return to its home (launch) position.
 * Parameters for the general command: Empty| Empty| Empty| Empty| Empty| Empty| Empty|
 *
 * @return true if the command is acknowledged; false otherwise.
 */
bool UNSCModule::returnToLaunchPosition() {
    // Needed here because it could be called while the Offboard mode is active
    cancelTimer(this->offboard_timer_);
    return this->sendToCommanderUnit(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
    );
}

/**
 * @brief Executes preflight checks to verify the simulation is ready for operation.
 */
void UNSCModule::runPreChecks() {
    // Delete wall timer to have it set off only once
    cancelTimer(this->prechecks_timer_);

    this->signalPublishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_RUN_PREARM_CHECKS);

    auto start_time =
        this->gatherTime().nanoseconds() / constants::NANO_TO_MILLI_CONVERSION; // [ms]
    this->sendLogDebug("Status data wait started at timestamp {}", start_time);

    // Check for a bit, in case the ACK has not been received yet
    while (true) {
        std::optional<px4_msgs::msg::VehicleStatus> status = this->gatherStatus();

        if (status) { // At least one status received
            this->sendLogDebug(
                "Stored status message found! timestamp:{} arming_state:{} nav_state:{} "
                "pre_flight_checks_pass:{} failsafe:{}",
                status->timestamp, status->arming_state, status->nav_state,
                status->pre_flight_checks_pass, status->failsafe
            );

            if (this->gatherTime().nanoseconds() / constants::NANO_TO_MILLI_CONVERSION >=
                start_time + constants::ACK_WAIT_MILLIS) {
                // Check whether the agent in the simulation is good to go
                std::lock_guard lock(this->sitl_ready_mutex_);
                this->sitl_ready_ =
                    status->pre_flight_checks_pass && !status->failsafe &&
                    ((1u << status->nav_state) != 0) && // Taken from the PX4 commander
                    status->arming_state < px4_msgs::msg::VehicleStatus::ARMING_STATE_STANDBY_ERROR;

                if (this->sitl_ready_)
                    this->sendLogInfo("Simulation ready!");
                else
                    this->sendLogError("Errors in the simulation detected!");

                return;
            } else {
                this->sendLogDebug("Status is too old (referring to {})", status->timestamp);
            }
        }

        // Wait for a bit before retrying
        std::this_thread::sleep_for(std::chrono::milliseconds(constants::DELAY_MILLIS));
    }

    this->sendLogDebug("Status data wait finished");

    // Default to the hypothesis that the simulation has some problem
    std::lock_guard lock(this->sitl_ready_mutex_);
    this->sitl_ready_ = false;
    this->sendLogError("Errors in the simulation detected!");
}

/**
 * @brief Commands the vehicle to enter a loiter mode.
 *
 * @return true if the command is acknowledged; false otherwise.
 */
bool UNSCModule::loiter() {
    return this->sendToCommanderUnit(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
        constants::MAVLINK_ENABLE_CUSTOM_MODE, constants::PX4_CUSTOM_MAIN_MODE,
        constants::PX4_LOITER_SUB_MODE
    );
}

/**
 * @brief Activates the consensus logic by setting up timers for offboard mode and rendezvous tasks.
 */
void UNSCModule::activateConsensus() {
    this->offboard_timer_ = this->node_->create_wall_timer(
        this->offboard_period_, std::bind(&UNSCModule::setAndMaintainOffboardMode, this),
        this->gatherOffboardExclusiveGroup()
    );
    this->rendezvous_timer_ = this->node_->create_wall_timer(
        this->rendezvous_period_, std::bind(&UNSCModule::consensusToRendezvous, this),
        this->gatherRendezvousExclusiveGroup()
    );
}

/**
 * @brief Sets and maintains the offboard control mode, publishing position and velocity setpoints.
 */
void UNSCModule::setAndMaintainOffboardMode() {
    auto maybe_pos = this->getPositionSetpoint();
    if (!maybe_pos) {
        this->sendLogDebug("No target pose found");
        return;
    }
    auto target_pos = maybe_pos.value();

    auto maybe_vel = this->getSetpointVelocity();
    if (!maybe_vel) {
        this->sendLogDebug("No target vel found");
        return;
    }
    auto target_vel = maybe_vel.value();

    Eigen::Vector3d des_body_pos = convertENUtoNED(
        {target_pos.x, target_pos.y, target_pos.z}, Eigen::Vector3d(this->getOffset().data())
    );
    // Velocity conversion from ENU to NED
    std::vector<double> des_body_vel = {target_vel.y, target_vel.x, -target_vel.z};

    if (this->offboard_setpoint_counter_ == constants::OFFBOARD_SETPOINT_LIMIT) {
        // Change to Offboard mode after the needed amount of setpoints
        this->sendLogDebug("Changing to Offboard mode!");
        this->signalPublishVehicleCommand(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
            constants::MAVLINK_ENABLE_CUSTOM_MODE, constants::PX4_OFFBOARD_MODE
        );
    }

    // offboard_control_mode needs to be paired with trajectory_setpoint
    this->signalPublishOffboardControlMode();
    this->signalPublishTrajectorySetpoint(
        geometry_msgs::msg::Point()
            .set__x(des_body_pos(0))
            .set__y(des_body_pos(1))
            .set__z(des_body_pos(2)),
        geometry_msgs::msg::Point()
            .set__x(des_body_vel[0])
            .set__y(des_body_vel[1])
            .set__z(des_body_vel[2])
    );

    // stop the counter after reaching OFFBOARD_SETPOINT_LIMIT + 1
    if (this->offboard_setpoint_counter_ <= constants::OFFBOARD_SETPOINT_LIMIT) {
        this->offboard_setpoint_counter_++;
    }
}

/**
 * @brief Sends a command to the PX4 commander and waits for acknowledgment.
 *
 * @param command The PX4 command to send.
 * @param param1 Parameter 1 of the command (default: NAN).
 * @param param2 Parameter 2 of the command (default: NAN).
 * @param param3 Parameter 3 of the command (default: NAN).
 * @param param4 Parameter 4 of the command (default: NAN).
 * @param param5 Parameter 5 of the command (default: NAN).
 * @param param6 Parameter 6 of the command (default: NAN).
 * @param param7 Parameter 7 of the command (default: NAN).
 * @return true if the command is acknowledged within the retry limit; false otherwise.
 */
bool UNSCModule::sendToCommanderUnit(
    uint16_t command, float param1, float param2, float param3, float param4, float param5,
    float param6, float param7
) {
    // Send command to PX4 commander and wait for it to acknowledge it
    unsigned int attempt = 1;
    do {
        this->sendLogDebug("Trying to send command {} and to get an ack", command);
        this->signalPublishVehicleCommand(
            command, param1, param2, param3, param4, param5, param6, param7
        );
        attempt++;
    } while ((!this->signalWaitForCommanderAck(command)) &&
             attempt < constants::MAX_SEND_COMMAND_RETRIES);

    if (attempt >= constants::MAX_SEND_COMMAND_RETRIES) {
        this->sendLogWarning("No ack received from the commander unit for command {}", command);
        return false;
    }

    this->sendLogInfo("Command {} sent", command);
    return true;
}
