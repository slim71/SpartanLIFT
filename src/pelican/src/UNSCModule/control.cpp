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

bool UNSCModule::loiter() {
    cancelTimer(this->offboard_timer_);
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
    auto opt_target = this->getSetpointPosition();
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

/*************************** Rendezvous ****************************/
// Collision avoidance is included in the updating algorithm
void UNSCModule::consensusToRendezvous() {
    auto maybe_des_pos = this->getTargetPosition();
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

    // Not changing the z component, so the current one is kept
    this->setSetpointPosition(updated_pos);
    this->setSetpointVelocity(vel);
}

void UNSCModule::preFormationActions() {
    // Non-leader agents simply start the formation control
    // The leader agent has some more instructions to follow, before starting the formation control

    // Move leader to target position
    if (this->confirmAgentIsLeader()) {
        std::optional<geometry_msgs::msg::Point> maybe_target = this->getTargetPosition();
        if (!maybe_target) {
            this->sendLogWarning("Target data not available!");
            return;
        }
        geometry_msgs::msg::Point target_pos = maybe_target.value();
        this->sendLogDebug("Positioning to the target position: {}", target_pos);
        this->setSetpointPosition(maybe_target.value());

        while (true) {
            this->sendLogDebug("Checking if target position has been reached");
            geometry_msgs::msg::Point pos = this->gatherCopterPosition(this->gatherAgentID());
            this->sendLogDebug("Target: {}, last recorded: {}", target_pos, pos);
            if ((abs(pos.x - target_pos.x) < constants::SETPOINT_REACHED_DISTANCE) &&
                (abs(pos.y - target_pos.y) < constants::SETPOINT_REACHED_DISTANCE)) {
                this->sendLogDebug("Reached");
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(constants::DELAY_MILLIS));
        }
    }

    // Descend
    this->sendLogDebug("Getting lower");
    this->signalSetReferenceHeight(2.0);
    while (true) {
        this->sendLogDebug("Checking if new target height has been reached");
        geometry_msgs::msg::Point pos = this->gatherCopterPosition(this->gatherAgentID());
        this->sendLogDebug("last height recorded: {}", pos.z);
        if (abs(pos.z - 2.0) < 0.1) {
            this->sendLogDebug("Reached");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(constants::DELAY_MILLIS));
    }

    // Attach cargo to leader
    if (this->confirmAgentIsLeader()) {
        this->sendLogDebug("Attaching cargo to leader");
        this->signalCargoAttachment();
    }

    // Start formation control
    this->sendLogDebug("Ready for formation control!");
    if (this->confirmAgentIsLeader()) {
        this->assignFormationPositions();
    } else {
        this->formation_timer_ = this->node_->create_wall_timer(
            this->formation_period_, // CHECK: maybe greater period
            std::bind(&UNSCModule::formationControl, this), this->gatherFormationTimerGroup()
        );
    }
}

geometry_msgs::msg::Point UNSCModule::adjustmentForCollisionAvoidance(
    geometry_msgs::msg::Point agentPosition, double threshold
) {
    // Initialize the total adjustment vector to zero
    double totalAdjustmentX = 0.0;
    double totalAdjustmentY = 0.0;

    // To reduce function calls
    unsigned int net_size = this->gatherNetworkSize();
    unsigned int my_id = this->gatherAgentID();

    // Compute adjustment vector for each neighbor
    for (unsigned int copter_id = 1; copter_id <= net_size; copter_id++) { // for each copter
        if (copter_id != my_id) {                   // Exclude my own position
            geometry_msgs::msg::Point neigh_position = this->gatherCopterPosition(copter_id);
            if (!geomPointHasNan(neigh_position)) { // Copter position valid

                // Compute vector from neighbor to agent
                double dx = neigh_position.x - agentPosition.x;
                double dy = neigh_position.y - agentPosition.y;
                // Euclidean distance between the agent and the neighbor
                double distance = std::hypot(dx, dy);
                this->sendLogDebug(
                    "2D distance from copter {} at {}: {:.4f} ({:.4f},{:.4f})", copter_id,
                    neigh_position, distance, dx, dy
                );

                // Adjust only if the distance is less than the threshold
                if (distance < threshold) {
                    // Compute the adjustment direction (away from the neighbor)
                    double adjustmentMagnitude = constants::AVOIDANCE_DISTANCE - distance;
                    double adjustmentDirectionX = dx / distance * adjustmentMagnitude;
                    double adjustmentDirectionY = dy / distance * adjustmentMagnitude;
                    this->sendLogDebug(
                        "Collision avoidance adjustment contribution: {:.4f}, {:.4f} (mag: {:.4f})",
                        adjustmentDirectionX, adjustmentDirectionY, adjustmentMagnitude
                    );

                    // Accumulate adjustments
                    totalAdjustmentX -= adjustmentDirectionX;
                    totalAdjustmentY -= adjustmentDirectionY;
                }
            } else {
                this->sendLogDebug(
                    "Neighbor {} position contains NANs ({})", copter_id, neigh_position
                );
            }
        }
    }
    // Return the normalized total adjustment vector
    return geometry_msgs::msg::Point().set__x(totalAdjustmentX).set__y(totalAdjustmentY);
}

// CHECK: right now I'm establishing the neighbors before actually giving them a desired position;
// does it cause issues? it could; let the leader inform them of their neighbors?
void UNSCModule::findNeighbors() {
    std::vector<unsigned int> ids = this->gatherCoptersIDs();
    auto own_pos = this->gatherCopterPosition(this->gatherAgentID());
    std::vector<std::pair<unsigned int, double>> distances;

    // Compute distance from each copter
    for (unsigned int id : ids) {
        if ((id != this->gatherAgentID()) && (id != this->gatherLeaderID())) {
            auto agent_pos = this->gatherCopterPosition(id);
            double distance = std::hypot(own_pos.x - agent_pos.x, own_pos.y - agent_pos.y);
            this->sendLogDebug("ID {} is at distance: {}", id, distance);
            distances.push_back(std::make_pair(id, distance));
        } else {
            this->sendLogDebug("ID {} is either mine or leader's", id);
            distances.push_back(std::make_pair(id, 1000.0));
        }
    }

    // Sort distance vector
    std::sort(
        distances.begin(), distances.end(),
        [](const std::pair<unsigned int, double> a, const std::pair<unsigned int, double> b) {
            return a.second < b.second;
        }
    );
    // Find neighbors based on minimum distances
    if (distances.size() >= 2) {
        this->neighbors_[0] = distances[0].first;
        this->neighbors_[1] = distances[1].first;
        this->sendLogDebug("Neighbors should be: {}, {}", this->neighbors_[0], this->neighbors_[1]);
    } else {
        this->sendLogDebug("Not enough neighbors found");
    }
}

/**************************** Formation ****************************/

void UNSCModule::formationControl() {
    if ((this->neighbors_[0] == UINT_MAX) || (this->neighbors_[1] == UINT_MAX)) {
        this->findNeighbors();
        return;
    }

    // CHECK: add id to FleetInfo?
    // Gather own and neighbors' current position
    geometry_msgs::msg::Point my_curr_pos = this->gatherCopterPosition(this->gatherAgentID());
    geometry_msgs::msg::Point my_des_pos = this->gatherDesiredPosition();
    this->sendLogDebug("Own current position: {}, desired position: {}", my_curr_pos, my_des_pos);
    if (geomPointHasNan(my_des_pos))
        return;
    geometry_msgs::msg::Point n0_pos =
        this->gatherCopterPosition(this->neighbors_[0]); // CHECK: treat in a vector?
    geometry_msgs::msg::Point n1_pos = this->gatherCopterPosition(this->neighbors_[1]);
    this->sendLogDebug("Neighbors' positions: {}, {}", n0_pos, n1_pos);
    if (geomPointHasNan(n0_pos) || geomPointHasNan(n1_pos))
        return;
    geometry_msgs::msg::Point leader_pos = this->gatherCopterPosition(this->gatherLeaderID());
    this->sendLogDebug("Leader (ID: {}) position: {}", this->gatherLeaderID(), leader_pos);
    if (geomPointHasNan(leader_pos))
        return;

    // Gather neighbors' desired position
    std::vector<geometry_msgs::msg::Point> neigh_des;
    for (auto&& neighbor : this->neighbors_) {
        this->formation_cv_mutex_.lock(); // TODO: setter function?
        this->neighbor_gathered_ = false;
        this->formation_cv_mutex_.unlock();

        // Call neighbors service for its desired position
        this->sendLogDebug("Handling neighbor {}", neighbor);
        this->signalAskPositionToNeighbor(neighbor);

        // Wait for the answer to arrive
        std::unique_lock lock(this->formation_cv_mutex_);
        this->formation_cv_.wait(lock, [this] {
            return this->neighbor_gathered_;
        });
        lock.unlock();

        // Add the neighbor's position in the vector
        geometry_msgs::msg::Point n_des_pos = this->gatherNeighborDesiredPosition();
        if (geomPointHasNan(n_des_pos)) {
            this->sendLogWarning("Neighbor's desired position is not valid");
            return; // Stop this iteration
        } else {
            neigh_des.push_back(n_des_pos);
            this->sendLogDebug("Neighbor's desired position is good: {}", n_des_pos);
        }
    }

    // Compute control input u_i = K_p * \sum_j (p_j - p_i - p_j^{desired} + p_i^{desired})
    // The leader current and desired position correspond, since it is guiding the fleet
    // CHECK: weight the contributions
    double ux = (my_curr_pos.x - n0_pos.x - (my_des_pos.x - neigh_des[0].x)) +
                (my_curr_pos.x - n1_pos.x - (my_des_pos.x - neigh_des[1].x)) +
                (my_curr_pos.x - leader_pos.x - (my_des_pos.x - leader_pos.x));
    double uy = (my_curr_pos.y - n0_pos.y - (my_des_pos.y - neigh_des[0].y)) +
                (my_curr_pos.y - n1_pos.y - (my_des_pos.y - neigh_des[1].y)) +
                (my_curr_pos.y - leader_pos.y - (my_des_pos.y - leader_pos.y));
    this->sendLogDebug(
        "current: {}, ux: {}, uy: {}, deltax: {}, deltay: {}", my_curr_pos, ux, uy,
        my_curr_pos.x - ux, my_curr_pos.y - uy
    );

    // Compute adjustment vector for collision avoidance with nearby agents
    geometry_msgs::msg::Point collision_adjustment =
        adjustmentForCollisionAvoidance(my_curr_pos, constants::FORMATION_COLLISION_THRESHOLD);
    this->sendLogDebug("Collision adjustment for formation control: {}", collision_adjustment);

    // Compute udpated reference position
    geometry_msgs::msg::Point new_pos =
        geometry_msgs::msg::Point()
            .set__x(my_curr_pos.x - ux / 10) // CHECK: this for the weight of the contributions
            .set__y(my_curr_pos.y - uy / 10)
            .set__z(my_curr_pos.z);
    this->sendLogDebug("New formation position to move to: {}", new_pos);

    // Apply influence of the adjustment vector for collision avoidance
    new_pos.x += constants::BETA_REND * collision_adjustment.x;
    new_pos.y += constants::BETA_REND * collision_adjustment.y;
    this->sendLogDebug("Updated formation position after collision adjustments: {}", new_pos);

    // Small velocity to help a more precise tracking (if needed)
    geometry_msgs::msg::Point vel = geometry_msgs::msg::Point()
                                        .set__x((my_des_pos.x - my_curr_pos.x) / 2)
                                        .set__y((my_des_pos.y - my_curr_pos.y) / 2)
                                        .set__z(std::nanf(""));
    this->sendLogDebug("Updated velocity for the last formation setpoint: {}", vel);

    this->setSetpointVelocity(vel);
    this->setSetpointPosition(new_pos);
}

void UNSCModule::assignFormationPositions() {
    std::vector<geometry_msgs::msg::Point> positions;
    double circle_radius = this->gatherROI();
    unsigned int my_id = this->gatherAgentID();        // to reduce function calls
    unsigned int net_size = this->gatherNetworkSize(); // to reduce function calls

    // Gather all copters' positions
    for (unsigned int i = 1; i <= net_size; i++) {
        positions.push_back(this->gatherCopterPosition(i));
    }

    // Compute the distance of each copter from my own, aka the
    // center of the desired circle, and find the closest one
    unsigned int closest_id = 0;
    double min_distance = std::numeric_limits<double>::max();
    for (unsigned int i = 0; i < net_size; ++i) {
        double dist = circleDistance(positions[my_id - 1], positions[i], circle_radius);
        if (dist < min_distance && i != my_id - 1) {
            min_distance = dist;
            closest_id = i + 1;
        }
        this->sendLogDebug("Agent {} position: {}, distance: {}", i, positions[i], dist);
    }

    // Determine closest point on circle
    auto closest_desired_pos =
        closestCirclePoint(positions[closest_id - 1], positions[my_id - 1], circle_radius);

    // Determine all desired positions on the circle
    std::vector<geometry_msgs::msg::Point> desired_positions =
        homPointsOnCircle(closest_desired_pos, positions[my_id - 1], circle_radius, net_size - 1);

    // Make sure the leader and the closest agent have the correct position
    desired_positions.push_back(positions[my_id - 1]); // Add leader position
    std::swap(
        desired_positions[0], desired_positions[closest_id - 1]
    ); // Move first position on circle to closest agent
    if (!((my_id == 1) && (closest_id == net_size))) {
        std::swap(desired_positions[my_id - 1], desired_positions[net_size - 1]);
    }

    // std::vector<int> assignment(positions.size(), -1);
    std::vector<bool> assigned(desired_positions.size(), false);

    // Compute closest desired position to each agent
    for (unsigned int i = 0; i < net_size; i++) {
        double min_distance = std::numeric_limits<double>::max();
        int min_index = -1;

        // Skip leader and "chosen" agent
        if ((i == my_id - 1) || (i == closest_id - 1)) {
            min_distance = 0;
            min_index = i;
        } else {
            for (unsigned int j = 0; j < desired_positions.size(); j++) {
                // Skip leader's and "chosen" agent's positions
                if (!assigned[j] && (j != my_id - 1) && (j != closest_id - 1)) {
                    double dist = p2p2DDistance(positions[i], desired_positions[j]);
                    if (dist < min_distance) {
                        min_distance = dist;
                        min_index = j;
                    }
                }
            }
        }

        std::swap(desired_positions[min_index], desired_positions[i]);
        assigned[min_index] = true;
    }

    // Output the sorted desired positions with distances
    for (unsigned int i = 0; i < positions.size(); ++i) {
        const auto& pos = desired_positions[i];
        double distance = p2p2DDistance(positions[i], pos);
        this->sendLogInfo(
            "Agent {} assigned to position: {} with distance: {}", i + 1, pos, distance
        );
    }

    this->signalSendFormationPositions(desired_positions);
}
