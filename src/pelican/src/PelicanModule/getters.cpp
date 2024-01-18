#include "PelicanModule/pelican.hpp"

unsigned int Pelican::getID() const {
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

bool Pelican::isReady() const {
    return this->ready_;
}

int Pelican::getNetworkSize() {
    std::lock_guard<std::mutex> lock(this->discovery_mutex_);
    auto s = this->discovery_vector_.size();
    this->sendLogInfo("Discovered network has size {}", s);
    return s;
}
