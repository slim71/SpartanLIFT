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
    // Needed here because it could be called while the Offboard mode is active
    cancelTimer(this->offboard_timer_);
    return this->sendToCommanderUnit(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
}

// Use current?| Empty| Empty| Empty| Latitude| Longitude| Altitude|
bool UNSCModule::setHome() {
    return this->sendToCommanderUnit(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_HOME, constants::CONFIRM_SET_HOME
    );
}

// Empty| Empty| Empty| Empty| Empty| Empty| Empty|
bool UNSCModule::returnToLaunchPosition() {
    // Needed here because it could be called while the Offboard mode is active
    cancelTimer(this->offboard_timer_);
    return this->sendToCommanderUnit(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
    );
}

void UNSCModule::runPreChecks() {
    // Delete wall timer to have it set off only once
    cancelTimer(this->prechecks_timer_);

    this->signalPublishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_RUN_PREARM_CHECKS);

    auto start_time =
        this->gatherTime().nanoseconds() / constants::NANO_TO_MILLI_CONVERSION; // [ms]
    this->sendLogDebug("Status data wait started at timestamp {}", start_time);

    // Check for a bit, in case the ACK has not been received yet
    while (this->gatherTime().nanoseconds() / constants::NANO_TO_MILLI_CONVERSION - start_time <
           constants::ACK_WAIT_MILLIS) {
        std::optional<px4_msgs::msg::VehicleStatus> status = this->gatherStatus();

        if (status) { // At least one status received
            this->sendLogDebug(
                "Stored status message found! timestamp:{} arming_state:{} nav_state:{} "
                "pre_flight_checks_pass:{} failsafe:{}",
                status->timestamp, status->arming_state, status->nav_state,
                status->pre_flight_checks_pass, status->failsafe
            );

            // Check whether the agent in the simulation is good to go
            this->sitl_ready_ =
                status->pre_flight_checks_pass && !status->failsafe &&
                ((1u << status->nav_state) != 0) && // Taken from the PX4 commander
                status->arming_state < px4_msgs::msg::VehicleStatus::ARMING_STATE_STANDBY_ERROR;

            if (this->sitl_ready_)
                this->sendLogInfo("Simulation ready!");
            else
                this->sendLogError("Errors in the simulation detected!");

            this->setHome();

            return;
        }

        // Wait for a bit before retrying
        std::this_thread::sleep_for(std::chrono::milliseconds(constants::DELAY_MILLIS));
    }

    this->sendLogDebug("Status data wait finished");

    // Default to the hypothesis that the simulation has some problem
    this->sitl_ready_ = false;
    this->sendLogError("Errors in the simulation detected!");
}

bool UNSCModule::loiter() {
    return this->sendToCommanderUnit(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
        constants::MAVLINK_ENABLE_CUSTOM_MODE, constants::PX4_CUSTOM_MAIN_MODE,
        constants::PX4_LOITER_SUB_MODE
    );
}

void UNSCModule::activateOffboardMode() {
    this->offboard_timer_ = this->node_->create_wall_timer(
        this->offboard_period_, std::bind(&UNSCModule::setAndMaintainOffboardMode, this),
        this->gatherOffboardExclusiveGroup()
    );
    this->rendezvous_timer_ = this->node_->create_wall_timer(
        this->rendezvous_period_, std::bind(&UNSCModule::consensusToRendezvous, this),
        this->gatherRendezvousExclusiveGroup()
    );
}

void UNSCModule::setAndMaintainOffboardMode() {
    auto opt_target = this->getPositionSetpoint();
    if (!opt_target) {
        this->sendLogDebug("No target pose found");
        return;
    }
    auto target_pos = opt_target.value();
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

        this->arm();
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

bool UNSCModule::sendToCommanderUnit(
    uint16_t command, float param1, float param2, float param3, float param4, float param5,
    float param6, float param7
) {
    // Send command to PX4 commander and wait for it to acknoledge it
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
