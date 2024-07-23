#include "PelicanModule/pelican.hpp"
#include "UNSCModule/unsc.hpp"

/*************************** Rendezvous ****************************/
void UNSCModule::consensusToRendezvous() {
    // Be sure to have all needed data
    std::optional<geometry_msgs::msg::Point> maybe_target_pos = this->getTargetPosition();
    std::optional<nav_msgs::msg::Odometry> maybe_odom = this->gatherENUOdometry();
    if (!maybe_odom || !maybe_target_pos) {
        this->sendLogWarning("Needed data not available!");
        return;
    }

    // Gather initial position of the agent and duplicate it
    // CHECK: use current NED measurement instead?
    geometry_msgs::msg::Point init_pos = this->gatherCopterPosition(this->gatherAgentID());
    geometry_msgs::msg::Point updated_pos = init_pos;
    geometry_msgs::msg::Point target_pos = maybe_target_pos.value();
    nav_msgs::msg::Odometry last_enu_odom = maybe_odom.value();
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

    // If close enough to target setpoint (updated position)
    if ((x_diff < constants::SETPOINT_REACHED_DISTANCE) &&
        (y_diff < constants::SETPOINT_REACHED_DISTANCE) &&
        (z_diff < constants::SETPOINT_REACHED_DISTANCE)) {
        cancelTimer(this->rendezvous_timer_); // Do not call this function again
        this->sendLogDebug("Finished Rendezvous successfully");

        this->preFormationActions();

        // The last setpoints update is not actually needed, so don't perform it
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
    if (this->confirmAgentIsLeader()) {
        std::optional<geometry_msgs::msg::Point> maybe_target = this->getTargetPosition();
        if (!maybe_target) {
            this->sendLogWarning("Target data not available!");
            return;
        }
        geometry_msgs::msg::Point target_pos = maybe_target.value();
        this->sendLogDebug("Positioning to the target position: {}", target_pos);
        this->setPositionSetpoint(maybe_target.value());

        while (true) {
            this->sendLogDebug("Checking if target position has been reached");
            geometry_msgs::msg::Point pos = this->gatherCopterPosition(this->gatherAgentID());
            this->sendLogDebug("Target: {}, last recorded: {}", target_pos, pos);
            if ((abs(target_pos.x - pos.x) < constants::SETPOINT_REACHED_DISTANCE) &&
                (abs(target_pos.y - pos.y) < constants::SETPOINT_REACHED_DISTANCE)) {
                this->sendLogDebug("Reached");
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
    }

    // Descend
    this->sendLogDebug("Getting lower");
    this->signalSetReferenceHeight(2.0);
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
    }

    // Start formation control
    this->sendLogDebug("Ready for formation control!");
    if (this->confirmAgentIsLeader()) {
        this->assignFormationPositions();
    } else {
        this->formation_timer_ = this->node_->create_wall_timer(
            this->formation_period_, std::bind(&UNSCModule::formationControl, this),
            this->gatherFormationExclusiveGroup()
        );
    }
}

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
    std::unordered_map<unsigned int, geometry_msgs::msg::Point> neigh_des =
        this->neigh_des_positions_;
    this->neighbors_despos_mutex_.unlock();

    // Make sure we have all neighbors' current position
    std::unordered_map<unsigned int, geometry_msgs::msg::Point> neigh_pos;
    for (auto& neighbor : this->getNeighborsIDs()) {
        auto p = this->gatherCopterPosition(neighbor);
        if (geomPointHasNan(p))
            return;
        neigh_pos[neighbor] = p;
        this->sendLogDebug("Neighbor {} is at {}", neighbor, p);
    }

    // Gather own current and desired position
    geometry_msgs::msg::Point my_curr_pos = this->gatherCopterPosition(this->gatherAgentID());
    geometry_msgs::msg::Point my_des_pos = this->gatherDesiredPosition();
    this->sendLogDebug("Own current position: {}, desired position: {}", my_curr_pos, my_des_pos);
    if (geomPointHasNan(my_des_pos))
        return;

    // Gather leader's current and desired position
    geometry_msgs::msg::Point leader_pos = this->gatherCopterPosition(this->gatherLeaderID());
    this->sendLogDebug("Leader (ID: {}) position: {}", this->gatherLeaderID(), leader_pos);
    if (geomPointHasNan(leader_pos))
        return;

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

    // Compute adjustment vector for collision avoidance with nearby agents
    geometry_msgs::msg::Point coll_adj =
        adjustmentForCollisionAvoidance(my_curr_pos, constants::FORM_COLL_THRESHOLD);
    this->sendLogDebug("Collision adjustment for formation control: {}", coll_adj);
    // Apply influence of the adjustment vector for collision avoidance
    new_pos.x += constants::FORM_COLL_WEIGHT * coll_adj.x;
    new_pos.y += constants::FORM_COLL_WEIGHT * coll_adj.y;
    this->sendLogDebug("Updated formation position after collision adjustments: {}", new_pos);

    // Small velocity to help a more precise tracking (if needed)
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

    this->setPositionSetpoint(new_pos);
    this->setVelocitySetpoint(vel);
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
                double distance = std::hypot(dx, dy);

                // Ensure distance is not zero to avoid division by zero
                if (distance == 0) {
                    this->sendLogWarning(
                        "Distance to copter {} is zero, skipping adjustment", copter_id
                    );
                    continue;
                }
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
    std::unordered_map<unsigned int, geometry_msgs::msg::Point> positions;
    double circle_radius = this->gatherROI();
    unsigned int my_id = this->gatherAgentID(); // to reduce function calls
    std::vector<unsigned int> ids = this->gatherCoptersIDs();

    // Gather all copters' positions
    for (auto& id : ids) {
        positions[id] = this->gatherCopterPosition(id);
    }

    // Compute the distance of each copter from my own, aka the
    // center of the desired circle, and find the closest one
    unsigned int closest_id = 0;
    double min_distance = std::numeric_limits<double>::max();
    for (auto& id : ids) {
        // Skip me
        if (id == my_id)
            continue;

        double dist = circleDistance(positions.at(my_id), positions.at(id), circle_radius);
        if (dist < min_distance) {
            min_distance = dist;
            closest_id = id;
        }
        this->sendLogDebug("Agent {} position: {}, distance: {}", id, positions.at(id), dist);
    }
    this->sendLogDebug("Closest agent is {}", closest_id);

    // Determine closest point on circle
    geometry_msgs::msg::Point closest_desired_pos =
        closestCirclePoint(positions.at(closest_id), positions.at(my_id), circle_radius);

    // Determine all desired positions on the circle
    std::vector<geometry_msgs::msg::Point> desired_positions = homPointsOnCircle(
        closest_desired_pos, positions.at(my_id), circle_radius, this->gatherNetworkSize() - 1
    );
    // Add leader position
    desired_positions.push_back(positions.at(my_id));

    std::unordered_map<unsigned int, geometry_msgs::msg::Point> associated_pos;
    // Make sure the leader and the closest agent have the correct position
    associated_pos[my_id] = positions.at(my_id);
    associated_pos[closest_id] = desired_positions[0]; // == closest_desired_pos

    std::vector<bool> assigned(desired_positions.size(), false);
    assigned[0] = true;
    assigned[assigned.size() - 1] = true;

    // Compute closest desired position to each agent
    for (auto& id : ids) {
        // Skip leader and "chosen" agent
        if ((id == my_id) || (id == closest_id))
            continue;

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
        assigned[min_index] = true;
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
    bool cv_timed_out = this->formation_cv_.wait_for(
        lock, std::chrono::seconds(constants::MAX_WAITING_TIME_SECS),
        [this] {
            return this->neighbor_gathered_;
        }
    );
    lock.unlock();

    geometry_msgs::msg::Point n_des_pos;

    // If wait_for timed out, behave as if the neighbor returned a NAN position
    if (cv_timed_out) {
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
