#include "PelicanModule/pelican.hpp"
#include "UNSCModule/unsc.hpp"

bool UNSCModule::arm() {
    return this->sendToCommanderUnit(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        px4_msgs::msg::VehicleCommand::ARMING_ACTION_ARM
    );
}

bool UNSCModule::disarm() {
    cancelTimer(this->offboard_timer_);
    return this->sendToCommanderUnit(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        px4_msgs::msg::VehicleCommand::ARMING_ACTION_DISARM
    );
}

// Pitch| Empty| Empty| Yaw| Latitude| Longitude| Altitude|
bool UNSCModule::takeoff(unsigned int height) {
    cancelTimer(this->offboard_timer_);
    return this->sendToCommanderUnit(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, NAN, NAN, NAN, NAN, NAN, NAN,
        (height > 0) ? height : NAN
    );
}

// Empty| Empty| Empty| Yaw| Latitude| Longitude| Altitude|
bool UNSCModule::land() {
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
    cancelTimer(this->offboard_timer_);
    return this->sendToCommanderUnit(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
    );
}

void UNSCModule::runPreChecks() {
    // Delete wall timer to have it set off only once
    cancelTimer(this->starting_timer_);

    this->signalPublishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_RUN_PREARM_CHECKS);

    auto start_time =
        this->gatherTime().nanoseconds() / constants::NANO_TO_MILLI_ORDER_CONVERSION; // [ms]
    this->sendLogDebug("Status data wait started at timestamp {}", start_time);

    // Check for a bit, in case the ACK has not been received yet
    while (this->gatherTime().nanoseconds() / constants::NANO_TO_MILLI_ORDER_CONVERSION -
               start_time <
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

void UNSCModule::activateOffboardMode() {
    this->offboard_timer_ = this->node_->create_wall_timer(
        this->offboard_period_, std::bind(&UNSCModule::setAndMaintainOffboardMode, this)
        // [this] {
        // this->setAndMaintainOffboardMode(); // TODO: use std::bind}
    );
    this->rendezvous_timer_ = this->node_->create_wall_timer(
        this->rendezvous_period_, std::bind(&UNSCModule::consensusToRendezvous, this)
    ); // TODO: how to terminate it when Offboard phase is finished
}

void UNSCModule::setAndMaintainOffboardMode() { // TODO: not with itself
    auto opt_target = this->gatherTargetPose();
    float x, y, z, yaw;
    if (opt_target) {
        auto target = opt_target.value();
        x = target[0];
        y = target[1];
        z = target[2];
        yaw = 0;
    } else {
        this->sendLogDebug("No target pose found");
        return;
    }
    auto maybe_vel = this->gatherTargetVelocity();
    float vx, vy;
    if (maybe_vel) {
        auto target = maybe_vel.value();
        vx = target[0];
        vy = target[1];
    } else {
        this->sendLogDebug("No target vel found");
        return;
    }

    Eigen::Vector3f des_body_pos = this->convertLocalToBody({x, y, z});
    std::vector<double> des_body_vel = {vy, vx}; // Inverted to convert from ENU to NED

    // TODO: delete
    this->sendLogDebug(
        "offboard: ({:.4f}, {:.4f}, {:.4f}) became ({:.4f}, {:.4f}, {:.4f})", x, y, z,
        des_body_pos(0), des_body_pos(1), des_body_pos(2)
    );
    this->sendLogDebug(
        "velocity ({:.4f}, {:.4f}) became ({:.4f}, {:4f})", vx, vy, des_body_vel[0], des_body_vel[1]
    );

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
        des_body_pos(0), des_body_pos(1), des_body_pos(2), yaw, des_body_vel[0], des_body_vel[1]
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

/**************************************************************************************/
void UNSCModule::consensusToRendezvous(
) { // collision avoidance is included in the updating algorithm
    double alpha = 0.2;
    double beta = 1.0;

    auto maybe_predefined = this->gatherDesiredPose();
    if (!maybe_predefined) {
        this->sendLogWarning("Desired not set!");
        return;
    }
    auto predefinedLocation = maybe_predefined.value();

    auto initialPosition = this->gatherCopterPosition(this->gatherAgentID());
    auto updatedPosition = initialPosition;
    this->sendLogDebug(
        "Own pos at start: ({:.4f},{:.4f},{:.4f})", updatedPosition.x, updatedPosition.y,
        updatedPosition.z
    );

    // Compute the distance between the agent and the predefined location
    double distanceToPredefinedLocation = std::hypot(
        updatedPosition.x - predefinedLocation[0], updatedPosition.y - predefinedLocation[1]
    );
    this->sendLogDebug(
        "distance To PredefinedLocation ({:.4f}, {:.4f}): {:.4f}", predefinedLocation[0],
        predefinedLocation[1], distanceToPredefinedLocation
    );

    // Compute adjustment vector for collision avoidance with nearby agents
    geometry_msgs::msg::Point adjustmentVector = adjustmentForCollisionAvoidance(updatedPosition);
    this->sendLogDebug(
        "adjustmentVector: ({:.4f},{:.4f},-)", adjustmentVector.x, adjustmentVector.y
    );

    // Check if the distance is less than the threshold
    if (distanceToPredefinedLocation >= 2.0) {
        // Apply influence of the predefined location (alpha)
        updatedPosition.x = alpha * predefinedLocation[0] + (1 - alpha) * updatedPosition.x;
        updatedPosition.y = alpha * predefinedLocation[1] + (1 - alpha) * updatedPosition.y;
        this->sendLogDebug(
            "Updated position after alpha: ({:.4f},{:.4f},-)", updatedPosition.x, updatedPosition.y
        );
    }

    // Apply influence of the adjustment vector for collision avoidance (beta)
    updatedPosition.x += beta * adjustmentVector.x;
    updatedPosition.y += beta * adjustmentVector.y;
    this->sendLogDebug(
        "Updated position after beta: ({:.4f},{:.4f},-)", updatedPosition.x, updatedPosition.y
    );

    if ((updatedPosition == initialPosition) && (adjustmentVector.x == 0.0) &&
        (adjustmentVector.y == 0.0)) {
        this->sendLogDebug("all ok");
        cancelTimer(this->rendezvous_timer_);
    }
    this->signalSetSetpointPosition(
        updatedPosition.x, updatedPosition.y, this->node_->getActualTargetHeight()
    ); // TODO: if this works, adapt the get function; not the actual, but the current
    std::vector<double> vel = {
        (updatedPosition.x - initialPosition.x) / 0.5,
        (updatedPosition.y - initialPosition.y) /
            0.5}; // TODO: the time is the same as the timer for consensusToRendezvous, in secs
    this->sendLogDebug("Computed velocity for next setpoint: {:.4f}, {:.4f}", vel[0], vel[1]);
    this->signalSetTargetVelocity(vel[0], vel[1]);
}

// TODO: clearer and more concise logs
geometry_msgs::msg::Point
UNSCModule::adjustmentForCollisionAvoidance(geometry_msgs::msg::Point agentPosition) {
    // Initialize the total adjustment vector to zero
    double totalAdjustmentX = 0.0;
    double totalAdjustmentY = 0.0;
    double avoidanceDistance = 1.0;

    unsigned int net_size = this->gatherNetworkSize();
    unsigned int my_id = this->gatherAgentID();

    // Compute adjustment vector for each neighbor
    for (unsigned int copter_id = 1; copter_id <= net_size; copter_id++) { // for each copter
        if (copter_id != my_id) { // exclude my own position
            auto neighborPosition = this->gatherCopterPosition(copter_id);
            if (neighborPosition !=
                NAN_point) { // copter position valid // CHECK: != not working with nan?

                // Compute vector from neighbor to agent
                double dx = neighborPosition.x - agentPosition.x;
                double dy = neighborPosition.y - agentPosition.y;
                double distance =
                    std::hypot(dx, dy); // Euclidean distance between the agent and the neighbor
                this->sendLogDebug(
                    "distance from copter {} at ({:.4f},{:.4f},{:.4f}): {:.4f} ({:.4f},{:.4f})",
                    copter_id, neighborPosition.x, neighborPosition.y, neighborPosition.z, distance,
                    dx, dy
                );

                // Adjust only if the distance is less than the threshold
                if (distance < avoidanceDistance) {
                    // Compute the adjustment direction (away from the neighbor)
                    double adjustmentMagnitude = avoidanceDistance - distance;
                    double adjustmentDirectionX = dx / distance * adjustmentMagnitude;
                    double adjustmentDirectionY = dy / distance * adjustmentMagnitude;
                    this->sendLogDebug(
                        "Adjustment contributions: {:.4f}, {:.4f} (mag: {:.4f})",
                        adjustmentDirectionX, adjustmentDirectionY, adjustmentMagnitude
                    );

                    // Accumulate adjustments
                    totalAdjustmentX -= adjustmentDirectionX;
                    totalAdjustmentY -= adjustmentDirectionY;
                }
            }
        }
    }

    geometry_msgs::msg::Point ret_value;
    ret_value.x = totalAdjustmentX;
    ret_value.y = totalAdjustmentY;
    ret_value.z = 0;

    // Return the normalized total adjustment vector
    return ret_value;
}
