#include "PelicanModule/pelican.hpp"

/************************* Standard getters ***************************/
unsigned int Pelican::getID() const {
    std::lock_guard lock(this->id_mutex_);
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
    std::lock_guard lock(this->term_mutex_);
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

rclcpp::CallbackGroup::SharedPtr Pelican::getFormationExclusiveGroup() const {
    return this->formation_exclusive_group_;
}

rclcpp::CallbackGroup::SharedPtr Pelican::getFormationTimerGroup() const {
    return this->formation_timer_group_;
}

rclcpp::Time Pelican::getTime() const {
    return this->now();
}

double Pelican::getROI() const {
    return this->roi_;
}

unsigned int Pelican::getNetworkSize() const {
    std::lock_guard lock(this->discovery_mutex_);
    int s = this->discovery_vector_.size() + 1;
    return s;
}

geometry_msgs::msg::Point Pelican::getCopterPosition(unsigned int id) const {
    try {
        return this->copters_positions_.at(id);
    } catch (const std::out_of_range&) {
        // ID not known
        return NAN_point;
    }
}

std::vector<unsigned int> Pelican::getCoptersIDs() const {
    std::vector<unsigned int> ids;
    std::lock_guard lock(this->discovery_mutex_);
    for (auto& elem : this->discovery_vector_) {
        this->sendLogDebug("Accruing ID {}", elem.agent_id);
        ids.push_back(elem.agent_id);
    }
    ids.push_back(this->getID()); // Ensure my ID is considered

    return ids;
}

geometry_msgs::msg::Point Pelican::getDesiredPosition() const {
    std::lock_guard lock(this->formation_mutex_);
    return this->des_formation_pos_;
}

geometry_msgs::msg::Point Pelican::getNeighborDesiredPosition() const {
    std::lock_guard lock(this->formation_mutex_);
    return this->neigh_des_pos_;
}

geometry_msgs::msg::Point Pelican::getDropoffPosition() const {
    std::lock_guard lock(this->dropoff_mutex_);
    return this->dropoff_position_;
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
    std::lock_guard lock(this->flying_mutex_);
    return this->flying_;
}

bool Pelican::isCarrying() const {
    std::lock_guard lock(this->carrying_mutex_);
    return this->carrying_;
}

bool Pelican::isLastCmdExecuted() const {
    std::lock_guard lock(this->last_cmd_result_mutex_);
    return this->last_cmd_result_;
}

bool Pelican::isFormationAchieved() const {
    std::lock_guard lock(this->form_achieved_mutex_);
    return this->formation_achieved_;
}
