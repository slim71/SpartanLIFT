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

void Pelican::setSetpointPosition(float x, float y, float z) {
    std::lock_guard<std::mutex> lock(this->setpoint_position_mutex_);
    if (this->setpoint_position_.size() < 3) {
        this->sendLogDebug("Target vector too small; resizing"); // TODO: delete?
        this->setpoint_position_.resize(3);
    }
    this->sendLogDebug("Setting target vector to ({:.4f},{:.4f},{:.4f})", x, y, z);
    this->setpoint_position_[0] = x;
    this->setpoint_position_[1] = y;
    if (!std::isnan(z)) {
        this->sendLogDebug("target z component is not nan");
        this->setpoint_position_[2] = z;
    }
}

void Pelican::setTargetVelocity(float vx, float vy) {
    std::lock_guard<std::mutex> lock(this->setpoint_velocity_mutex_);
    if (this->setpoint_velocity_.size() < 2) {
        this->sendLogDebug("Target velocity too small; resizing"); // TODO: delete?
        this->setpoint_velocity_.resize(2);
    }
    this->sendLogDebug("Setting target vector to ({:.4f},{:.4f})", vx, vy);
    this->setpoint_velocity_[0] = vx;
    this->setpoint_velocity_[1] = vy;
}

void Pelican::setTargetPosition(float x, float y, float z) {
    std::lock_guard<std::mutex> lock(this->target_position_mutex_);
    if (this->target_position_.size() < 3) {
        this->sendLogDebug("Desired vector too small; resizing"); // TODO: delete?
        this->target_position_.resize(3);
    }
    this->sendLogDebug("Setting desired vector to ({:.4f},{:.4f},{:.4f})", x, y, z);
    this->target_position_[0] = x;
    this->target_position_[1] = y;
    if (!std::isnan(z)) {
        this->sendLogDebug("target z component is not nan");
        this->target_position_[2] = z;
    }
}

void Pelican::setReferenceHeight(float height) {
    std::lock_guard<std::mutex> lock(this->height_mutex_);
    this->actual_target_height_ = height;
}
