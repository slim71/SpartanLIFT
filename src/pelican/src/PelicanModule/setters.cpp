#include "PelicanModule/pelican.hpp"

void Pelican::setRole(possible_roles r) {
    this->sendLogDebug("Setting role to {}", roles_to_string(r));
    this->role_ = r;
}

void Pelican::setMass(double m) {
    this->mass_ = m;
}

void Pelican::setInstance(rclcpp::Node::SharedPtr instance) {
    instance_ = std::static_pointer_cast<Pelican>(instance);
}

void Pelican::setTerm(unsigned int t) {
    this->current_term_ = t;
    this->sendLogInfo("Updated term to {}", this->current_term_);
}

void Pelican::commenceIncreaseCurrentTerm() {
    this->current_term_++;
    this->sendLogInfo("New term: {}", this->current_term_);
}

void Pelican::commenceSetTerm(uint64_t term) {
    this->current_term_ = term;
    this->sendLogInfo("New term: {}", this->current_term_);
}
