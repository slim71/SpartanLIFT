#include "PelicanModule/pelican.hpp"
#include "UNSCModule/unsc.hpp"

/*************************** Rendezvous ****************************/
void UNSCModule::consensusToRendezvous() {
    // Be sure to have all needed data
    std::optional<geometry_msgs::msg::Point> maybe_target_pos = this->getTargetPosition();
    if (!maybe_target_pos) {
        this->sendLogWarning("Needed data not available!");
        return;
    }

    // Gather initial position of the agent and duplicate it
    geometry_msgs::msg::Point init_pos = this->gatherCopterPosition(this->gatherAgentID());
    geometry_msgs::msg::Point updated_pos = init_pos;
    geometry_msgs::msg::Point target_pos = maybe_target_pos.value();
    double target_height = this->getActualTargetHeight();
    this->sendLogDebug("Own pos at start of rendezvous iteration: {}", init_pos);

    // Compute the distance between the agent and the target location
    double dist_to_target_pos = std::hypot(init_pos.x - target_pos.x, init_pos.y - target_pos.y);
    this->sendLogDebug(
        "Distance to desired rendezvous position {}: {:.4f}", target_pos, dist_to_target_pos
    );

    // If I'm not yet inside a ROI around the rendezvous point
    if (dist_to_target_pos >= this->gatherROI()) {
        // Apply influence of the target location
        updated_pos.x = constants::TARGET_LOC_WEIGHT * target_pos.x +
                        (1 - constants::TARGET_LOC_WEIGHT) * init_pos.x;
        updated_pos.y = constants::TARGET_LOC_WEIGHT * target_pos.y +
                        (1 - constants::TARGET_LOC_WEIGHT) * init_pos.y;
        this->sendLogDebug("Updated rendezvous position after distance coverage: {}", updated_pos);
    }

    // Compute adjustment vector for collision avoidance with nearby agents
    geometry_msgs::msg::Point coll_adj =
        adjustmentForCollisionAvoidance(updated_pos, constants::REND_COLL_THRESHOLD);
    updated_pos.x += constants::REND_COLL_WEIGHT * coll_adj.x;
    updated_pos.y += constants::REND_COLL_WEIGHT * coll_adj.y;
    this->sendLogDebug("Updated rendezvous position after collision adjustments: {}", updated_pos);

    // Compute distance from setpoint
    auto x_diff = abs(updated_pos.x - init_pos.x);
    auto y_diff = abs(updated_pos.y - init_pos.y);
    auto z_diff = abs(target_height - init_pos.z);

    // If the new position is close enough to target setpoint, go to the next step
    if ((x_diff <= constants::SETPOINT_REACHED_DISTANCE) &&
        (y_diff <= constants::SETPOINT_REACHED_DISTANCE) &&
        (z_diff <= constants::SETPOINT_REACHED_DISTANCE)) {
        cancelTimer(this->rendezvous_timer_); // Do not call this function again
        this->sendLogDebug("Finished Rendezvous successfully");

        // The last setpoints update is not actually needed; PX4 will manage the stabilization on
        // its own
        return;
    }

    geometry_msgs::msg::Point vel =
        geometry_msgs::msg::Point()
            .set__x(
                (updated_pos.x - init_pos.x) /
                (constants::RENDEZVOUS_PERIOD_MILLIS / constants::MILLIS_TO_SECS_CONVERSION)
            )
            .set__y(
                (updated_pos.y - init_pos.y) /
                (constants::RENDEZVOUS_PERIOD_MILLIS / constants::MILLIS_TO_SECS_CONVERSION)
            )
            .set__z(std::nanf("")); // Do not handle the z axis (height)
    this->sendLogDebug("Computed next velocity setpoint for rendezvous operations: {}", vel);

    // Not changing the z component, so the current one is kept
    this->setPositionSetpoint(updated_pos);
    this->setVelocitySetpoint(vel);
}

/**************************** Formation ****************************/
void UNSCModule::preFormationActions() {
    // Non-leader agents simply start the formation control
    // The leader agent has some more instructions to follow, before starting the formation control

    // Move leader to target position
    // TODO: also add collision avoidance to the leader movements
    if (this->confirmAgentIsLeader()) {
        std::optional<geometry_msgs::msg::Point> maybe_target = this->getTargetPosition();
        if (!maybe_target) {
            this->sendLogWarning("Target data not available!");
            return;
        }
        geometry_msgs::msg::Point target_pos = maybe_target.value();
        this->sendLogDebug("Positioning to the target position: {}", target_pos);
        this->setPositionSetpoint(target_pos);
        double target_height = this->getActualTargetHeight();

        while (true) {
            this->sendLogDebug("Checking if target position has been reached");
            geometry_msgs::msg::Point pos = this->gatherCopterPosition(this->gatherAgentID());
            this->sendLogDebug("Target: {}, last recorded: {}", target_pos, pos);
            if ((abs(target_pos.x - pos.x) <= constants::SETPOINT_REACHED_DISTANCE) &&
                (abs(target_pos.y - pos.y) <= constants::SETPOINT_REACHED_DISTANCE) &&
                (abs(target_height - pos.z) <= constants::SETPOINT_REACHED_DISTANCE)) {
                this->sendLogDebug("Reached");
                this->signalSyncTrigger();
                break;
            } else {
                geometry_msgs::msg::Point vel =
                    geometry_msgs::msg::Point()
                        .set__x(
                            (target_pos.x - pos.x) /
                            (constants::DELAY_MILLIS / constants::MILLIS_TO_SECS_CONVERSION)
                        )
                        .set__y(
                            (target_pos.y - pos.y) /
                            (constants::DELAY_MILLIS / constants::MILLIS_TO_SECS_CONVERSION)
                        )
                        .set__z(std::nanf("")); // Do not handle the z axis (height)
                this->sendLogDebug(
                    "Computed next velocity setpoint for pre-formation operations: {}", vel
                );
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(constants::DELAY_MILLIS));
        }
    } else {
        this->waitForSyncCount();
        this->decreaseSyncCount();
    }

    // Descend
    this->sendLogDebug("Getting lower");
    this->signalSetReferenceHeight(constants::EXTRACTION_HEIGHT);
    while (true) {
        this->sendLogDebug("Checking if new target height has been reached");
        geometry_msgs::msg::Point pos = this->gatherCopterPosition(this->gatherAgentID());
        this->sendLogDebug("Last height recorded: {}", pos.z);
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
    } else {
        this->waitForSyncCount();
        this->decreaseSyncCount();
    }

    // Make the agents fly higher
    this->sendLogDebug("Getting higher");
    this->signalSetReferenceHeight(constants::EXTRACTION_HEIGHT + 1);
    while (true) {
        this->sendLogDebug("Checking if new target height has been reached");
        geometry_msgs::msg::Point pos = this->gatherCopterPosition(this->gatherAgentID());
        this->sendLogDebug("Last height recorded: {}", pos.z);
        if (abs(pos.z - 3.0) < 0.1) {
            this->sendLogDebug("Reached");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(constants::DELAY_MILLIS));
    }

    // Start actual formation control
    this->sendLogDebug("Ready for formation control!");
    if (this->confirmAgentIsLeader()) {
        this->assignFormationPositions(); // To quickly send the first reference

        std::this_thread::sleep_for(std::chrono::seconds(constants::FORMATION_WAIT_TIME_SECS));

        this->formation_timer_ = this->node_->create_wall_timer(
            this->formation_period_, std::bind(&UNSCModule::assignFormationPositions, this),
            this->gatherFormationExclusiveGroup()
        );
    } else {
        this->sendLogDebug("Calling formationControl");
        this->formation_timer_ = this->node_->create_wall_timer(
            this->formation_period_, std::bind(&UNSCModule::formationControl, this),
            this->gatherFormationExclusiveGroup()
        );
    }

    // Move the leader to the dropoff position
    if (this->confirmAgentIsLeader()) {
        while (!this->confirmFormationAchieved()) {
            this->sendLogDebug("Formation not yet achieved");
            std::this_thread::sleep_for(std::chrono::milliseconds(constants::DELAY_MILLIS));
        }
        this->linear_timer_ = this->node_->create_wall_timer(
            this->linear_period_, std::bind(&UNSCModule::linearP2P, this),
            this->gatherFormationExclusiveGroup()
        );
    }
}

void UNSCModule::linearP2P() {
    // Gather dropoff position
    geometry_msgs::msg::Point dropoff_pos = this->gatherDropoffPosition();
    // Gather current position
    geometry_msgs::msg::Point curr_pos = this->gatherCopterPosition(this->gatherAgentID());
    // Gather target height
    double target_height = this->getActualTargetHeight();

    geometry_msgs::msg::Point diff = dropoff_pos - curr_pos;
    if ((abs(diff.x) <= constants::SETPOINT_REACHED_DISTANCE) &&
        (abs(diff.y) <= constants::SETPOINT_REACHED_DISTANCE) &&
        (abs(target_height - curr_pos.z) <= constants::SETPOINT_REACHED_DISTANCE)) {
        this->increaseTargetCount();
        this->sendLogDebug("Near target enough; count: {}", this->getTargetCount());
    } else {
        this->sendLogDebug("Distance from dropoff_pos: {}", diff);
        this->resetTargetCount();

        // Compute distance and normalize it
        double dx = dropoff_pos.x - curr_pos.x;
        double dy = dropoff_pos.y - curr_pos.y;
        double distance = std::hypot(dx, dy);
        // Compute intermediate setpoint
        geometry_msgs::msg::Point next_setpoint =
            geometry_msgs::msg::Point()
                .set__x(curr_pos.x + dx / distance * constants::STEP_SIZE)
                .set__y(curr_pos.y + dy / distance * constants::STEP_SIZE)
                .set__z(std::nanf("")); // Do not handle the z axis (height)
        this->sendLogDebug("Computed P2P position: {}", next_setpoint);

        // Add collision avoidance
        geometry_msgs::msg::Point coll_adj =
            adjustmentForCollisionAvoidance(next_setpoint, constants::FORM_COLL_THRESHOLD);
        next_setpoint.x += constants::REND_COLL_WEIGHT * coll_adj.x;
        next_setpoint.y += constants::REND_COLL_WEIGHT * coll_adj.y;
        this->sendLogDebug("Updated P2P position after collision adjustments: {}", next_setpoint);

        geometry_msgs::msg::Point vel =
            geometry_msgs::msg::Point()
                .set__x(
                    (next_setpoint.x - curr_pos.x) /
                    (constants::P2P_PERIOD_MILLIS / constants::MILLIS_TO_SECS_CONVERSION)
                )
                .set__y(
                    (next_setpoint.y - curr_pos.y) /
                    (constants::P2P_PERIOD_MILLIS / constants::MILLIS_TO_SECS_CONVERSION)
                )
                .set__z(std::nanf("")); // Do not handle the z axis (height)
        this->sendLogDebug("Computed next velocity setpoint for pre-formation operations: {}", vel);

        this->setPositionSetpoint(next_setpoint);
        this->setVelocitySetpoint(vel, constants::FORM_CLOSING_VEL);
    }
    if (this->getTargetCount() >= constants::FORM_TARGET_COUNT) {
        this->sendLogDebug("Static formation achieved successfully");
        cancelTimer(this->linear_timer_); // Do not call this function again
    }
}

// Only executed by non-leaders
void UNSCModule::formationControl() {
    /************** Preparations ***************/
    // Make sure neighbors are determined
    if (this->getNeighborsIDs().size() < 1) {
        this->sendLogDebug("Still missing some neighbors...");
        this->findNeighbors();
        return;
    }

    // Make sure we also have the desired positions of each neighbor
    this->neighbors_despos_mutex_.lock();
    // Pairs: neighbor ID - neighbor's desired position
    std::unordered_map<unsigned int, geometry_msgs::msg::Point> neigh_des =
        this->neigh_des_positions_;
    this->neighbors_despos_mutex_.unlock();

    // Make sure we have all neighbors' current position
    // Pairs: neighbor ID - neighbor's current position
    std::unordered_map<unsigned int, geometry_msgs::msg::Point> neigh_pos;
    for (auto& neighbor : this->getNeighborsIDs()) {
        auto p = this->gatherCopterPosition(neighbor);
        if (geomPointHasNan(p))
            return;
        neigh_pos[neighbor] = p;
        this->sendLogDebug("Neighbor {} is at {}", neighbor, p);
    }

    // Gather own current and desired position
    auto maybe_curr_pos = this->gatherENUOdometry();
    if (!maybe_curr_pos) {
        this->sendLogWarning("Needed data not available!");
        return;
    }
    auto enu = maybe_curr_pos.value();
    geometry_msgs::msg::Point my_curr_pos = geometry_msgs::msg::Point()
                                                .set__x(enu.pose.pose.position.x)
                                                .set__y(enu.pose.pose.position.y)
                                                .set__z(enu.pose.pose.position.z);
    geometry_msgs::msg::Point my_des_pos = this->gatherDesiredPosition();
    this->sendLogDebug("Own current position: {}, desired position: {}", my_curr_pos, my_des_pos);
    if (geomPointHasNan(my_des_pos))
        return;

    // Gather leader's current and desired position
    geometry_msgs::msg::Point leader_pos = this->gatherCopterPosition(this->gatherLeaderID());
    this->sendLogDebug("Leader (ID: {}) position: {}", this->gatherLeaderID(), leader_pos);
    if (geomPointHasNan(leader_pos))
        return;
    // Gather target height
    double target_height = this->getActualTargetHeight();

    /************* Actual algorith *************/
    // Compute control input u_i = K_p * \sum_j (p_j - p_i - (p_j^{desired} - p_i^{desired}))
    double ux = 0, uy = 0;
    // Neighbors' contribution
    for (auto& neighbor : this->getNeighborsIDs()) {
        ux += my_curr_pos.x - neigh_pos.at(neighbor).x - (my_des_pos.x - neigh_des.at(neighbor).x);
        uy += my_curr_pos.y - neigh_pos.at(neighbor).y - (my_des_pos.y - neigh_des.at(neighbor).y);
    }
    // Leader's contribution
    // The leader current and desired position correspond, since it is guiding the fleet
    ux += my_curr_pos.x - leader_pos.x - (my_des_pos.x - leader_pos.x);
    uy += my_curr_pos.y - leader_pos.y - (my_des_pos.y - leader_pos.y);
    // Account for the gain
    ux *= constants::FORM_DIST_WEIGHT;
    uy *= constants::FORM_DIST_WEIGHT;
    this->sendLogDebug("current: {}, ux: {}, uy: {}", my_curr_pos, ux, uy);

    // Compute udpated reference position
    geometry_msgs::msg::Point new_pos = geometry_msgs::msg::Point()
                                            .set__x(my_curr_pos.x - ux)
                                            .set__y(my_curr_pos.y - uy)
                                            .set__z(my_curr_pos.z); // Not generally used
    this->sendLogDebug("New formation position to move to: {}", new_pos);

    // Introduce a maximum step size to prevent large movements
    double dx = new_pos.x - my_curr_pos.x;
    double dy = new_pos.y - my_curr_pos.y;
    double step = std::sqrt(dx * dx + dy * dy);
    if (step > constants::STEP_SIZE) {
        new_pos.x = my_curr_pos.x + (dx / step) * constants::STEP_SIZE;
        new_pos.y = my_curr_pos.y + (dy / step) * constants::STEP_SIZE;
    }

    // Compute adjustment vector for collision avoidance with nearby agents
    geometry_msgs::msg::Point coll_adj = adjustmentForCollisionAvoidance(new_pos, 1);
    this->sendLogDebug("Collision adjustment for formation control: {}", coll_adj);
    // Apply influence of the adjustment vector for collision avoidance
    new_pos.x += constants::FORM_COLL_WEIGHT * coll_adj.x;
    new_pos.y += constants::FORM_COLL_WEIGHT * coll_adj.y;
    this->sendLogDebug("Updated formation position after collision adjustments: {}", new_pos);

    geometry_msgs::msg::Point vel =
        geometry_msgs::msg::Point()
            .set__x(
                (my_des_pos.x - my_curr_pos.x) /
                (constants::FORMATION_PERIOD_MILLIS / constants::MILLIS_TO_SECS_CONVERSION)
            )
            .set__y(
                (my_des_pos.y - my_curr_pos.y) /
                (constants::FORMATION_PERIOD_MILLIS / constants::MILLIS_TO_SECS_CONVERSION)
            )
            .set__z(std::nanf(""));
    this->sendLogDebug("Updated velocity for last formation setpoint: {}", vel);

    // Compute distance from desired position
    auto x_diff = abs(my_des_pos.x - my_curr_pos.x);
    auto y_diff = abs(my_des_pos.y - my_curr_pos.y);
    auto z_diff = abs(target_height - my_curr_pos.z);

    // If the new position is close enough to the target setpoint, go to the next step
    if ((x_diff <= constants::SETPOINT_REACHED_DISTANCE) &&
        (y_diff <= constants::SETPOINT_REACHED_DISTANCE) &&
        (z_diff <= constants::SETPOINT_REACHED_DISTANCE)) {
        this->increaseTargetCount();
        this->sendLogDebug(
            "Formation|Near enough to the desired position; count: {}", this->getTargetCount()
        );

        // Small velocity to help a more precise tracking (if needed)
        this->setVelocitySetpoint(vel, 0.2);

    } else {
        this->sendLogDebug("Formation|Distance from desired position: {}, {}", x_diff, y_diff);
        this->resetTargetCount();

        this->setVelocitySetpoint(vel);
    }

    if (this->getTargetCount() >= constants::FORM_TARGET_COUNT) {
        this->sendLogDebug("Formation achieved successfully");
        this->signalNotifyAgentInFormation();
    }

    this->setPositionSetpoint(new_pos);
}

/*********************** Collision avoidance ***********************/
geometry_msgs::msg::Point UNSCModule::adjustmentForCollisionAvoidance(
    geometry_msgs::msg::Point agentPosition, double threshold
) {
    // Initialize the total adjustment vector to zero
    double totalAdjustmentX = 0.0;
    double totalAdjustmentY = 0.0;

    // To reduce function calls
    unsigned int my_id = this->gatherAgentID();
    std::vector<unsigned int> ids = this->gatherCoptersIDs();

    // Compute adjustment vector for each neighbor
    for (auto& copter_id : ids) { // for each copter
        // Exclude my own position
        if (copter_id != my_id) {
            geometry_msgs::msg::Point neigh_position = this->gatherCopterPosition(copter_id);

            if (!geomPointHasNan(neigh_position)) { // Copter position valid
                // Euclidean distance between the agent and the neighbor
                double dx = neigh_position.x - agentPosition.x;
                double dy = neigh_position.y - agentPosition.y;
                // No problem: never exactly zero, since my own position is excluded
                double distance = std::hypot(dx, dy);

                this->sendLogDebug(
                    "2D distance from copter {} at {}: {:.4f} ({:.4f},{:.4f})", copter_id,
                    neigh_position, distance, dx, dy
                );

                // Adjust only if the distance is less than the threshold
                if (distance < threshold) {
                    // Compute the adjustment direction (away from the neighbor)
                    double adjustmentMagnitude = threshold - distance;
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
                    "Cannot compute collision contribution: neighbor {} position contains NANs "
                    "({})",
                    copter_id, neigh_position
                );
            }
        }
    }
    // Return the normalized total adjustment vector
    return geometry_msgs::msg::Point().set__x(totalAdjustmentX).set__y(totalAdjustmentY);
}

/************************** Neighborhood ***************************/
void UNSCModule::findNeighbors() {
    std::vector<unsigned int> ids = this->gatherCoptersIDs();
    geometry_msgs::msg::Point own_pos = this->gatherCopterPosition(this->gatherAgentID());
    std::multimap<double, unsigned int> distances;

    // Compute distance from each copter
    for (auto& id : ids) {
        // Exclude leader's and own ID
        if ((id != this->gatherAgentID()) && (id != this->gatherLeaderID())) {
            this->collectNeighDesPositions(id);
            geometry_msgs::msg::Point agent_des_pos = this->getNeighborDesPos(id);
            if (geomPointHasNan(agent_des_pos))
                return;
            double distance = std::hypot(own_pos.x - agent_des_pos.x, own_pos.y - agent_des_pos.y);
            this->sendLogDebug("Agent {} is at distance: {}", id, distance);
            distances.insert(std::pair<double, unsigned int>(distance, id));
        } else {
            this->sendLogDebug("ID {} is either mine or leader's", id);
            distances.insert(std::pair<double, unsigned int>(1000.0, id));
        }
    }

    // 2 agents are present for sure: the leader and me
    // I want at least one more (case of a 3-agents fleet)
    if (distances.size() <= 2) {
        this->sendLogWarning("Not enough agents found, so let's say I have no neighbors");
        return;
    }

    // Find neighbors based on minimum distances
    std::multimap<double, unsigned int>::iterator itr = distances.begin();

    this->neighbors_mutex_.lock();
    this->neighbors_.push_back(itr->second); // First neighbor

    if (this->gatherNetworkSize() > 3) {
        std::advance(itr, 1);
        this->neighbors_.push_back(itr->second); // Second neighbor
    }

    std::string output =
        fmt::format("Neighbors should be {}: {}", this->neighbors_.size(), this->neighbors_[0]);
    output += (this->neighbors_.size() > 1) ? fmt::format(" {}", this->neighbors_[1]) : "";
    this->sendLogDebug(output);
    this->neighbors_mutex_.unlock();
}

void UNSCModule::assignFormationPositions() {
    // Pairs: agent ID - agent's position
    std::unordered_map<unsigned int, geometry_msgs::msg::Point> positions;
    double circle_radius = this->gatherROI();
    unsigned int my_id = this->gatherAgentID(); // to reduce function calls
    std::vector<unsigned int> ids = this->gatherCoptersIDs();

    // Gather all copters' positions
    for (auto& id : ids) {
        positions[id] = this->gatherCopterPosition(id);
    }
    // Get my latest position, for more up-to-date computations
    auto maybe_enu = this->gatherENUOdometry();
    if (!maybe_enu) {
        this->sendLogDebug("ENU odometry not available!");
        return;
    }
    auto enu = maybe_enu.value();
    positions.at(my_id) = geometry_msgs::msg::Point()
                              .set__x(enu.pose.pose.position.x)
                              .set__y(enu.pose.pose.position.y)
                              .set__z(enu.pose.pose.position.z);

    if (this->getClosestAgent() == 0) {
        // Compute the distance of each copter from my own, aka the
        // center of the desired circle, and find the closest one
        unsigned int closest_id = 0;
        double min_distance = std::numeric_limits<double>::max();
        for (auto& id : ids) {
            // Skip me
            if (id == my_id)
                continue;

            double dist = circleDistance(positions.at(id), positions.at(my_id), circle_radius);
            if (dist < min_distance) {
                min_distance = dist;
                closest_id = id;
            }
            this->sendLogDebug(
                "Agent {} position: {}, circle distance: {}", id, positions.at(id), dist
            );
        }
        this->setClosestAgent(closest_id);
        this->setFleetOrder(closest_id, 0);
        this->setClosestAngle(std::atan2(positions.at(closest_id).y, positions.at(closest_id).x));
        this->sendLogDebug("Closest agent is {}, at angle {}", closest_id, this->getClosestAngle());
    }

    // Determine all desired positions on the circle
    std::vector<geometry_msgs::msg::Point> desired_positions = homPointsOnCircle(
        this->getClosestAngle(), positions.at(my_id), circle_radius, this->gatherNetworkSize() - 1
    );
    // Add leader position
    desired_positions.push_back(positions.at(my_id));
    this->setFleetOrder(my_id, desired_positions.size() - 1);

    // Pairs: agent ID - associated position
    std::unordered_map<unsigned int, geometry_msgs::msg::Point> associated_pos;
    // Make sure the leader and the closest agent have the correct position
    associated_pos[my_id] = positions.at(my_id);
    associated_pos[this->getClosestAgent()] = desired_positions[0];

    // Some presets...
    std::vector<bool> assigned(desired_positions.size(), false);
    assigned[0] = true;
    assigned[assigned.size() - 1] = true;

    // Compute closest desired position to each agent
    for (auto& id : ids) {
        // Skip leader and "chosen" agent
        if ((id == my_id) || (id == this->getClosestAgent()))
            continue;

        // Element-agent pair not yet created
        if (this->safeOrderFind(id)) {
            double min_distance = std::numeric_limits<double>::max();
            int min_index = -1;

            // Associate each remaining position to an agent
            for (unsigned int j = 0; j < desired_positions.size(); j++) {
                // Skip leader's and "chosen" agent's positions
                if (!assigned[j]) {
                    double dist = p2p2DDistance(positions.at(id), desired_positions[j]);
                    if (dist < min_distance) {
                        min_distance = dist;
                        min_index = j;
                    }
                }
            }

            associated_pos[id] = desired_positions[min_index];
            this->setFleetOrder(id, min_index);
            assigned[min_index] = true;
        } else {
            associated_pos[id] = desired_positions[this->getFleetOrder(id)];
        }
    }

    // Output the sorted desired positions with distances
    for (auto& id : ids) {
        double distance = p2p2DDistance(positions.at(id), associated_pos.at(id));
        this->sendLogInfo(
            "Agent {} assigned to position: {} with distance: {}", id, associated_pos.at(id),
            distance
        );
    }

    this->signalSendDesiredFormationPositions(associated_pos);
}

void UNSCModule::collectNeighDesPositions(unsigned int neighbor) {
    // Gather neighbor's desired position
    this->unsetNeighborGathered();

    // Call neighbors service for its desired position
    this->sendLogDebug("Collecting neighbor {}'s desired position", neighbor);
    this->signalAskDesPosToNeighbor(neighbor);

    // Wait for the answer to arrive
    std::unique_lock lock(this->formation_cv_mutex_);
    bool cv_successful = this->formation_cv_.wait_for(
        lock, std::chrono::seconds(constants::MAX_WAITING_TIME_SECS),
        [this] {
            return this->neighbor_gathered_;
        }
    );
    lock.unlock();

    geometry_msgs::msg::Point n_des_pos;

    // If wait_for timed out, behave as if the neighbor returned a NAN position
    if (!cv_successful) {
        this->sendLogError("Desired position not received for agent {}", neighbor);
        n_des_pos = NAN_point;
    } else {
        n_des_pos = this->gatherNeighborDesiredPosition();
        this->sendLogDebug(
            "Neighbor's {} desired position is {}: {}", neighbor,
            geomPointHasNan(n_des_pos) ? "not valid" : "good", n_des_pos
        );
    }

    // Add the neighbor's position in the vector
    this->neighbors_despos_mutex_.lock();
    this->neigh_des_positions_[neighbor] = n_des_pos;
    this->neighbors_despos_mutex_.unlock();
}
