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
    geometry_msgs::msg::Point collision_adjustment =
        adjustmentForCollisionAvoidance(updated_pos, this->gatherCollisionRadius());
    updated_pos.x += constants::REND_COLL_WEIGHT * collision_adjustment.x;
    updated_pos.y += constants::REND_COLL_WEIGHT * collision_adjustment.y;
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
                (updated_pos.x - init_pos.x) / (constants::RENDEZVOUS_CONSENSUS_PERIOD_MILLIS /
                                                constants::MILLIS_TO_SECS_CONVERSION)
            )
            .set__y(
                (updated_pos.y - init_pos.y) / (constants::RENDEZVOUS_CONSENSUS_PERIOD_MILLIS /
                                                constants::MILLIS_TO_SECS_CONVERSION)
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

/*********************** Collision avoidance ***********************/
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

void UNSCModule::formationControl() { // CHECK: to see if it improvable
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
        this->unsetNeighborGathered();

        // Call neighbors service for its desired position
        this->sendLogDebug("Handling neighbor {}", neighbor);
        this->signalAskDesPosToNeighbor(neighbor);

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

    // Compute control input u_i = K_p * \sum_j (p_j - p_i - (p_j^{desired} - p_i^{desired}))
    // The leader current and desired position correspond, since it is guiding the fleet
    double ux = constants::FORM_DIST_WEIGHT *
                ((my_curr_pos.x - n0_pos.x - (my_des_pos.x - neigh_des[0].x)) +
                 (my_curr_pos.x - n1_pos.x - (my_des_pos.x - neigh_des[1].x)) +
                 (my_curr_pos.x - leader_pos.x - (my_des_pos.x - leader_pos.x)));
    double uy = constants::FORM_DIST_WEIGHT *
                ((my_curr_pos.y - n0_pos.y - (my_des_pos.y - neigh_des[0].y)) +
                 (my_curr_pos.y - n1_pos.y - (my_des_pos.y - neigh_des[1].y)) +
                 (my_curr_pos.y - leader_pos.y - (my_des_pos.y - leader_pos.y)));
    this->sendLogDebug("current: {}, ux: {}, uy: {}", my_curr_pos, ux, uy);

    // Compute udpated reference position
    geometry_msgs::msg::Point new_pos = geometry_msgs::msg::Point()
                                            .set__x(my_curr_pos.x - ux)
                                            .set__y(my_curr_pos.y - uy)
                                            .set__z(my_curr_pos.z); // CHECK: more recent one?
    this->sendLogDebug("New formation position to move to: {}", new_pos);

    // Compute adjustment vector for collision avoidance with nearby agents
    geometry_msgs::msg::Point collision_adjustment =
        adjustmentForCollisionAvoidance(my_curr_pos, constants::FORM_COLL_THRESHOLD);
    this->sendLogDebug("Collision adjustment for formation control: {}", collision_adjustment);

    // Apply influence of the adjustment vector for collision avoidance
    new_pos.x += constants::FORM_COLL_WEIGHT * collision_adjustment.x;
    new_pos.y += constants::FORM_COLL_WEIGHT * collision_adjustment.y;
    this->sendLogDebug("Updated formation position after collision adjustments: {}", new_pos);

    // Small velocity to help a more precise tracking (if needed)
    geometry_msgs::msg::Point vel =
        geometry_msgs::msg::Point()
            .set__x(
                (my_des_pos.x - my_curr_pos.x) / (constants::RENDEZVOUS_CONSENSUS_PERIOD_MILLIS /
                                                  constants::MILLIS_TO_SECS_CONVERSION)
            )
            .set__y(
                (my_des_pos.y - my_curr_pos.y) / (constants::RENDEZVOUS_CONSENSUS_PERIOD_MILLIS /
                                                  constants::MILLIS_TO_SECS_CONVERSION)
            )
            .set__z(std::nanf(""));
    this->sendLogDebug("Updated velocity for last formation setpoint: {}", vel);

    this->setPositionSetpoint(new_pos);
    this->setVelocitySetpoint(vel);
}

/************************** Neighborhood ***************************/
// CHECK: right now I'm establishing the neighbors before actually giving them a desired position;
// does it cause issues? it could; let the leader inform them of their neighbors?
void UNSCModule::findNeighbors() {
    std::vector<unsigned int> ids = this->gatherCoptersIDs();
    geometry_msgs::msg::Point own_pos = this->gatherCopterPosition(this->gatherAgentID());
    std::vector<std::pair<unsigned int, double>> distances;

    // Compute distance from each copter
    for (unsigned int id : ids) {
        // Exclude leader's and own ID
        if ((id != this->gatherAgentID()) && (id != this->gatherLeaderID())) {
            geometry_msgs::msg::Point agent_pos = this->gatherCopterPosition(id);
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

    // Determine all desired positions on the circle // CHECK: provide IDs too, to be sure of ID-pos
    // pairing?
    std::vector<geometry_msgs::msg::Point> desired_positions =
        homPointsOnCircle(closest_desired_pos, positions[my_id - 1], circle_radius, net_size - 1);

    // Make sure the leader and the closest agent have the correct position
    // Add leader position
    desired_positions.push_back(positions[my_id - 1]);
    // Move first position on circle to closest agent
    std::swap(desired_positions[0], desired_positions[closest_id - 1]);
    if (!((my_id == 1) && (closest_id == net_size))) {
        std::swap(desired_positions[my_id - 1], desired_positions[net_size - 1]);
    }

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

    this->signalSendDesiredFormationPositions(desired_positions);
}
