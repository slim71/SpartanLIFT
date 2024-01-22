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

int Pelican::getNetworkSize() const {
    std::lock_guard<std::mutex> lock(this->discovery_mutex_);
    auto s = this->discovery_vector_.size();
    this->sendLogDebug("Discovered network has size {}", s);
    return s;
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
