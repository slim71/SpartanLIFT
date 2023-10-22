#include "PelicanModule/pelican.hpp"
#include "types.hpp"

void Pelican::setRole(possible_roles r) {
    this->sendLogDebug("Setting role to {}", roles_to_string(r));
    this->role_ = r;
}

void Pelican::commenceIncreaseCurrentTerm() {
    this->current_term_++;
}

void Pelican::setMass(double m) {
    this->mass_ = m;
}

void Pelican::setInstance(rclcpp::Node::SharedPtr instance) {
    instance_ = std::static_pointer_cast<Pelican>(instance);
}
