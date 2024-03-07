#include "PelicanModule/pelican.hpp"

/************************* Standard setters ***************************/
void Pelican::setRole(possible_roles r) {
    this->sendLogDebug("Setting role to {}", roles_to_string(r));
    this->role_ = r;
    this->logger_.cacheRole(r);
}

void Pelican::setMass(double m) {
    this->mass_ = m;
}

void Pelican::setInstance(rclcpp::Node::SharedPtr instance) {
    instance_ = std::static_pointer_cast<Pelican>(instance);
}

void Pelican::setTerm(unsigned int t) {
    this->sendLogInfo("Updating term to {}", t);
    this->logger_.cacheTerm(t);
    std::lock_guard<std::mutex> lock(this->term_mutex_);
    this->current_term_ = t;
}

void Pelican::setID(unsigned int id) {
    std::lock_guard<std::mutex> lock(this->id_mutex_);
    this->id_ = id;
}

void Pelican::setFlyingStatus() {
    std::lock_guard<std::mutex> lock(this->flying_mutex_);
    this->flying_ = true;
}

void Pelican::unsetFlyingStatus() {
    std::lock_guard<std::mutex> lock(this->flying_mutex_);
    this->flying_ = false;
}

void Pelican::setCarryingStatus() {
    std::lock_guard<std::mutex> lock(this->carrying_mutex_);
    this->carrying_ = true;
}

void Pelican::unsetCarryingStatus() {
    std::lock_guard<std::mutex> lock(this->carrying_mutex_);
    this->carrying_ = false;
}

void Pelican::setTargetPosition(float x, float y, float z) {
    std::lock_guard<std::mutex> lock(this->target_position_mutex_);
    this->target_position_ = {x, y, z};
}
