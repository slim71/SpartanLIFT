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

rclcpp::Time Pelican::getTime() const {
    return this->now();
}

float Pelican::getROI() const {
    return this->roi_;
}

int Pelican::getNetworkSize() const {
    std::lock_guard<std::mutex> lock(this->discovery_mutex_);
    auto s = this->discovery_vector_.size() + 1;
    this->sendLogDebug(
        "Discovered network has size {}", s
    ); // TODO: move somewhere else to log only once
    return s;
}

std::optional<std::vector<float>> Pelican::getTargetPosition() const {
    std::lock_guard<std::mutex> lock(this->target_position_mutex_);
    if (this->target_position_.size() > 0) {
        return this->target_position_;
    }

    return std::nullopt;
}

float Pelican::getActualTargetHeight() const {
    std::lock_guard<std::mutex> lock(this->height_mutex_);
    return this->actual_target_height_;
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
