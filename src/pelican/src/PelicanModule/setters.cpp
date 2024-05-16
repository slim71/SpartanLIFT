#include "PelicanModule/pelican.hpp"

/************************* Standard setters ***************************/
void Pelican::setRole(possible_roles r) {
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
    this->logger_.cacheTerm(t);
    std::lock_guard lock(this->term_mutex_);
    this->current_term_ = t;
}

void Pelican::setID(unsigned int id) {
    std::lock_guard lock(this->id_mutex_);
    this->id_ = id;
}

void Pelican::setFlyingStatus() {
    std::lock_guard lock(this->flying_mutex_);
    this->flying_ = true;
}

void Pelican::unsetFlyingStatus() {
    std::lock_guard lock(this->flying_mutex_);
    this->flying_ = false;
}

void Pelican::setCarryingStatus() {
    std::lock_guard lock(this->carrying_mutex_);
    this->carrying_ = true;
}

void Pelican::unsetCarryingStatus() {
    std::lock_guard lock(this->carrying_mutex_);
    this->carrying_ = false;
}

void Pelican::setLastCmdStatus() {
    std::lock_guard lock(this->last_cmd_result_mutex_);
    this->last_cmd_result_ = true;
}

void Pelican::unsetLastCmdStatus() {
    std::lock_guard lock(this->last_cmd_result_mutex_);
    this->last_cmd_result_ = false;
}

void Pelican::setAndNotifyRendezvousHandled() {
    std::lock_guard rd_lock(this->rendez_tristate_mutex_);
    this->rendezvous_handled_ = TriState::True;
    this->rend_handled_cv_.notify_all();
}

void Pelican::unsetAndNotifyRendezvousHandled() {
    std::lock_guard rd_lock(this->rendez_tristate_mutex_);
    this->rendezvous_handled_ = TriState::False;
    this->rend_handled_cv_.notify_all();
}
