#include "PelicanModule/pelican.hpp"

/************************* Standard getters ***************************/
unsigned int Pelican::getID() const {
    std::lock_guard<std::mutex> lock(this->id_mutex_);
    return this->id_;
}

std::string Pelican::getModel() const {
    return this->model_;
}

double Pelican::getMass() const {
    return this->mass_;
}

possible_roles Pelican::getRole() const {
    return this->role_;
}

unsigned int Pelican::getCurrentTerm() const {
    std::lock_guard<std::mutex> lock(this->term_mutex_);
    return this->current_term_;
}

std::shared_ptr<Pelican> Pelican::getInstance() {
    return instance_.lock();
}

rclcpp::SubscriptionOptions Pelican::getReentrantOptions() const {
    return this->reentrant_opt_;
}

rclcpp::CallbackGroup::SharedPtr Pelican::getReentrantGroup() const {
    return this->reentrant_group_;
}

rclcpp::CallbackGroup::SharedPtr Pelican::getTimerExclusiveGroup() const {
    return this->timer_exclusive_group_;
}

rclcpp::CallbackGroup::SharedPtr Pelican::getOffboardExclusiveGroup() const {
    return this->offboard_exclusive_group_;
}

rclcpp::CallbackGroup::SharedPtr Pelican::getRendezvousExclusiveGroup() const {
    return this->rendezvous_exclusive_group_;
}

rclcpp::Time Pelican::getTime() const {
    return this->now();
}

double Pelican::getROI() const {
    return this->roi_;
}

double Pelican::getCollisionRadius() const {
    return this->collision_radius_;
}

unsigned int Pelican::getNetworkSize() const {
    std::lock_guard<std::mutex> lock(this->discovery_mutex_);
    int s = this->discovery_vector_.size() + 1;
    return s;
}

std::optional<geometry_msgs::msg::Point> Pelican::getSetpointPosition() const {
    std::lock_guard<std::mutex> lock(this->setpoint_position_mutex_);
    if (this->setpoint_position_ != NAN_point) {
        return this->setpoint_position_;
    }

    return std::nullopt;
}

std::optional<geometry_msgs::msg::Point> Pelican::getSetpointVelocity() const {
    std::lock_guard<std::mutex> lock(this->setpoint_velocity_mutex_);
    if (this->setpoint_velocity_ != NAN_point) {
        return this->setpoint_velocity_;
    }

    return std::nullopt;
}

std::optional<geometry_msgs::msg::Point> Pelican::getTargetPosition() const {
    std::lock_guard<std::mutex> lock(this->target_position_mutex_);
    if (this->target_position_ != NAN_point) {
        return this->target_position_;
    }

    return std::nullopt;
}

double Pelican::getActualTargetHeight() const {
    std::lock_guard<std::mutex> lock(this->height_mutex_);
    return this->actual_target_height_;
}

geometry_msgs::msg::Point Pelican::getCopterPosition(unsigned int id) const {
    // Each node does not count itself in this, so manual +1 needed
    this->discovery_mutex_.lock();
    auto disc_len = this->discovery_vector_.size() + 1;
    this->discovery_mutex_.unlock();

    std::lock_guard<std::mutex> lock(this->positions_mutex_);
    auto copt_len = this->copters_positions_.size();

    // At least one needed vector is too short
    if ((disc_len < id) || (copt_len < id)) {
        return NAN_point;
    }

    return this->copters_positions_[id - 1];
}

/*************************** Status flags *****************************/
bool Pelican::isLeader() const {
    return (this->getRole() == leader);
}

bool Pelican::isFollower() const {
    return (this->getRole() == follower);
}

bool Pelican::isCandidate() const {
    return (this->getRole() == candidate);
}

bool Pelican::isReady() const {
    return this->ready_;
}

bool Pelican::isFlying() const {
    std::lock_guard<std::mutex> lock(this->flying_mutex_);
    return this->flying_;
}

bool Pelican::isCarrying() const {
    std::lock_guard<std::mutex> lock(this->carrying_mutex_);
    return this->carrying_;
}

bool Pelican::isLastCmdExecuted() const {
    std::lock_guard<std::mutex> lock(this->last_cmd_result_mutex_);
    return this->last_cmd_result_;
}
