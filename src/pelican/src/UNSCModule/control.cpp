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
    cancelTimer(this->prechecks_timer_);

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
        this->offboard_period_, std::bind(&UNSCModule::setAndMaintainOffboardMode, this),
        this->gatherOffboardExclusiveGroup()
    );
    this->rendezvous_timer_ = this->node_->create_wall_timer(
        this->rendezvous_period_, std::bind(&UNSCModule::consensusToRendezvous, this),
        this->gatherRendezvousExclusiveGroup()
    );
}

void UNSCModule::setAndMaintainOffboardMode() {
    auto opt_target = this->gatherSetpointPosition();
    if (!opt_target) {
        this->sendLogDebug("No target pose found");
        return;
    }
    auto target_pos = opt_target.value();
    auto maybe_vel = this->gatherSetpointVelocity();
    if (!maybe_vel) {
        this->sendLogDebug("No target vel found");
        return;
    }
    auto target_vel = maybe_vel.value();
    double vx = target_vel.x;
    double vy = target_vel.y;

    Eigen::Vector3d des_body_pos = convertENUtoNED(
        {target_pos.x, target_pos.y, target_pos.z}, Eigen::Vector3d(this->getOffset().data())
    );
    std::vector<double> des_body_vel = {vy, vx}; // Inverted to convert from ENU to NED

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
        geometry_msgs::msg::Point().set__x(des_body_vel[0]).set__y(des_body_vel[1])
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

bool UNSCModule::loiter() {
    cancelTimer(this->offboard_timer_);
    return this->sendToCommanderUnit(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
        constants::MAVLINK_ENABLE_CUSTOM_MODE, constants::PX4_CUSTOM_MAIN_MODE,
        constants::PX4_LOITER_SUB_MODE
    );
}

/**************************************************************************************/
// collision avoidance is included in the updating algorithm
void UNSCModule::consensusToRendezvous() {
    auto maybe_des_pos = this->gatherDesiredPosition();
    if (!maybe_des_pos) {
        this->sendLogWarning("Desired not set!");
        return;
    }
    auto des_pos = maybe_des_pos.value();

    auto init_pos = this->gatherCopterPosition(this->gatherAgentID());
    auto updated_pos = init_pos;
    this->sendLogDebug("Own pos at start of rendezvous iteration: {}", updated_pos);

    // Compute the distance between the agent and the predefined location
    double dist_to_des_pos = std::hypot(updated_pos.x - des_pos.x, updated_pos.y - des_pos.y);
    this->sendLogDebug("Distance to desired position {}: {:.4f}", des_pos, dist_to_des_pos);

    // Compute adjustment vector for collision avoidance with nearby agents
    geometry_msgs::msg::Point collision_adjustment = adjustmentForCollisionAvoidance(updated_pos);
    this->sendLogDebug("Collision adjustment: {}", collision_adjustment);

    // Check if the distance is less than the threshold
    if (dist_to_des_pos >= this->gatherROI()) {
        // Apply influence of the predefined location (constants::ALPHA_REND)
        updated_pos.x =
            constants::ALPHA_REND * des_pos.x + (1 - constants::ALPHA_REND) * updated_pos.x;
        updated_pos.y =
            constants::ALPHA_REND * des_pos.y + (1 - constants::ALPHA_REND) * updated_pos.y;
        this->sendLogDebug("Updated position after distance coverage: {}", updated_pos);
    }

    // Apply influence of the adjustment vector for collision avoidance
    updated_pos.x += constants::BETA_REND * collision_adjustment.x;
    updated_pos.y += constants::BETA_REND * collision_adjustment.y;
    this->sendLogDebug("Updated position after collision adjustments: {}", updated_pos);

    geometry_msgs::msg::Point vel = geometry_msgs::msg::Point()
                                        .set__x(
                                            (updated_pos.x - init_pos.x) /
                                            (constants::RENDEZVOUS_CONSENSUS_PERIOD_MILLIS / 1000.0)
                                        )
                                        .set__y(
                                            (updated_pos.y - init_pos.y) /
                                            (constants::RENDEZVOUS_CONSENSUS_PERIOD_MILLIS / 1000.0)
                                        );
    this->sendLogDebug("Computed velocity for next setpoint: {}", vel);

    // Stop offboard mode
    if (updated_pos == init_pos) {
        this->sendLogDebug("Stop position updates during rendezvous");
        cancelTimer(this->rendezvous_timer_); // Stop updating the setpoint

        // Timer for additional check in order to stop the
        // velocity setpoints update
        this->rend_check_timer_ = this->node_->create_wall_timer(
            this->rend_check_period_, std::bind(&UNSCModule::rendezvousClosure, this),
            this->gatherRendezvousExclusiveGroup()
        );
    }

    // Not changing the z component, so the current one is kept
    this->signalSetSetpointPosition(updated_pos);
    this->signalSetSetpointVelocity(vel);
}

void UNSCModule::rendezvousClosure() {
    auto maybe_setpoint = this->gatherSetpointPosition(); // target setpoint
    auto maybe_odom = this->gatherENUOdometry();          // Latest odometry data in ENU frame

    // If all data is available
    if (maybe_odom && maybe_setpoint) {
        geometry_msgs::msg::Point setpoint = maybe_setpoint.value();
        nav_msgs::msg::Odometry last_enu_odom = maybe_odom.value();
        auto target_height = this->gatherActualTargetHeight();
        this->sendLogDebug("target: {} current:{}", setpoint, last_enu_odom.pose.pose.position);

        // Compute distance from setpoint
        auto x_diff = abs(setpoint.x - last_enu_odom.pose.pose.position.x);
        auto y_diff = abs(setpoint.y - last_enu_odom.pose.pose.position.y);
        auto z_diff = abs(target_height - last_enu_odom.pose.pose.position.z);

        // If near target, stops Offboard mode for Rendezvous
        if ((x_diff < constants::SETPOINT_REACHED_DISTANCE) &&
            (y_diff < constants::SETPOINT_REACHED_DISTANCE) &&
            (z_diff < constants::SETPOINT_REACHED_DISTANCE)) {
            this->sendLogDebug("Finished rendezvous succesfully");
            cancelTimer(this->rend_check_timer_);
            std::lock_guard lock(this->rendezvous_cv_mutex_);
            this->rendezvous_done_ = true;
            this->rendezvous_cv_.notify_all();
            this->offboard_setpoint_counter_ = 0;

        } else {
            // Compute new velocity to aid movements towards target
            this->sendLogDebug(
                "still not finished -> x:{:.4f} y:{:.4f} z:{:.4f}", x_diff, y_diff, z_diff
            );

            geometry_msgs::msg::Point vel =
                geometry_msgs::msg::Point()
                    .set__x((setpoint.x - last_enu_odom.pose.pose.position.x) / 2)
                    .set__y((setpoint.y - last_enu_odom.pose.pose.position.y) / 2);
            this->sendLogDebug("Updated velocity for last setpoint: {}", vel);
            this->signalSetSetpointVelocity(vel);
        }
    }
}

geometry_msgs::msg::Point
UNSCModule::adjustmentForCollisionAvoidance(geometry_msgs::msg::Point agentPosition) {
    // Initialize the total adjustment vector to zero
    double totalAdjustmentX = 0.0;
    double totalAdjustmentY = 0.0;

    unsigned int net_size = this->gatherNetworkSize();
    unsigned int my_id = this->gatherAgentID();

    // Compute adjustment vector for each neighbor
    for (unsigned int copter_id = 1; copter_id <= net_size; copter_id++) { // for each copter
        if (copter_id != my_id) { // Exclude my own position
            auto neighborPosition = this->gatherCopterPosition(copter_id);
            // CHECK: != not working with nan? retest when seen again
            if (neighborPosition != NAN_point) { // Copter position valid

                // Compute vector from neighbor to agent
                double dx = neighborPosition.x - agentPosition.x;
                double dy = neighborPosition.y - agentPosition.y;
                // Euclidean distance between the agent and the neighbor
                double distance = std::hypot(dx, dy);
                this->sendLogDebug(
                    "2D distance from copter {} at {}: {:.4f} ({:.4f},{:.4f})", copter_id,
                    neighborPosition, distance, dx, dy
                );

                // Adjust only if the distance is less than the threshold
                if (distance < gatherCollisionRadius()) {
                    // Compute the adjustment direction (away from the neighbor)
                    double adjustmentMagnitude = constants::AVOIDANCE_DISTANCE - distance;
                    double adjustmentDirectionX = dx / distance * adjustmentMagnitude;
                    double adjustmentDirectionY = dy / distance * adjustmentMagnitude;
                    this->sendLogDebug(
                        "Adjustment contribution: {:.4f}, {:.4f} (mag: {:.4f})",
                        adjustmentDirectionX, adjustmentDirectionY, adjustmentMagnitude
                    );

                    // Accumulate adjustments
                    totalAdjustmentX -= adjustmentDirectionX;
                    totalAdjustmentY -= adjustmentDirectionY;
                }
            }
        }
    }
    // Return the normalized total adjustment vector
    return geometry_msgs::msg::Point().set__x(totalAdjustmentX).set__y(totalAdjustmentY);
    ;
}
