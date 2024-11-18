/**
 * @file setters.cpp
 * @author Simone Vollaro (slim71sv@gmail.com)
 * @brief File containing setter methods for the PelicanModule class.
 * @version 1.0.0
 * @date 2024-11-13
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "PelicanModule/pelican.hpp"

/************************* Standard setters ***************************/
/**
 * @brief Set the current role of the agent and notify the logger.
 *
 * @param r New role of the agent.
 */
void Pelican::setRole(possible_roles r) {
    this->role_ = r;
    this->logger_.cacheRole(r);
}

/**
 * @brief Record the mass of the agent.
 *
 * @param m Mass of the agent, gathered from the model used.
 */
void Pelican::setMass(double m) {
    this->mass_ = m;
}

/**
 * @brief Set the instance linked to the Pelican object.
 *
 * @param instance Pelican instance to link.
 */
void Pelican::setInstance(rclcpp::Node::SharedPtr instance) {
    instance_ = std::static_pointer_cast<Pelican>(instance);
}

/**
 * @brief Set new term ID.
 *
 * @param t Term ID.
 */
void Pelican::setTerm(unsigned int t) {
    this->logger_.cacheTerm(t);
    std::lock_guard lock(this->term_mutex_);
    this->current_term_ = t;
}

/**
 * @brief Set agent ID.
 *
 * @param id ID to set.
 */
void Pelican::setID(unsigned int id) {
    std::lock_guard lock(this->id_mutex_);
    this->id_ = id;
}

/**
 * @brief Record neighbor position.
 *
 * @param p Position of the neighbor.
 */
void Pelican::setNeighborPosition(geometry_msgs::msg::Point p) {
    std::lock_guard lock(this->neigh_mutex_);
    this->neigh_des_pos_ = p;
}

/**
 * @brief Record payload dropoff position.
 *
 * @param p Dropoff position.
 */
void Pelican::setDropoffPosition(geometry_msgs::msg::Point p) {
    std::lock_guard lock(this->dropoff_mutex_);
    this->dropoff_position_ = p;
}

/**************************** Flag setters ****************************/
/**
 * @brief Set the flying_ flag to True.
 */
void Pelican::setFlyingStatus() {
    std::lock_guard lock(this->flying_mutex_);
    this->flying_ = true;
}

/**
 * @brief Set the flying_ flag to False.
 */
void Pelican::unsetFlyingStatus() {
    std::lock_guard lock(this->flying_mutex_);
    this->flying_ = false;
}

/**
 * @brief Set the carrying_ flag to True.
 */
void Pelican::setCarryingStatus() {
    std::lock_guard lock(this->carrying_mutex_);
    this->carrying_ = true;
}

/**
 * @brief Set the carrying_ flag to False.
 */
void Pelican::unsetCarryingStatus() {
    std::lock_guard lock(this->carrying_mutex_);
    this->carrying_ = false;
}

/**
 * @brief Set the last_cmd_result_ flag to True.
 *
 * The value of the flag indicates whether the last command has been accepted by the fleet or not.
 */
void Pelican::setLastCmdStatus() {
    std::lock_guard lock(this->last_cmd_result_mutex_);
    this->last_cmd_result_ = true;
}

/**
 * @brief Set the last_cmd_result_ flag to False.
 *
 * The value of the flag indicates whether the last command has been accepted by the fleet or not.
 */
void Pelican::unsetLastCmdStatus() {
    std::lock_guard lock(this->last_cmd_result_mutex_);
    this->last_cmd_result_ = false;
}

/**
 * @brief Set the rendezvous_handled_ object to True.
 */
void Pelican::setAndNotifyRendezvousHandled() {
    std::lock_guard rd_lock(this->rendez_tristate_mutex_);
    this->rendezvous_handled_ = TriState::True;
    this->rendezvous_handled_cv_.notify_all();
}

/**
 * @brief Set the rendezvous_handled_ object to False.
 */
void Pelican::unsetAndNotifyRendezvousHandled() {
    std::lock_guard rd_lock(this->rendez_tristate_mutex_);
    this->rendezvous_handled_ = TriState::False;
    this->rendezvous_handled_cv_.notify_all();
}

/**
 * @brief Set the formation_handled_ object to True.
 */
void Pelican::setAndNotifyFormationHandled() {
    std::lock_guard rd_lock(this->form_tristate_mutex_);
    this->formation_handled_ = TriState::True;
    this->formation_handled_cv_.notify_all();
}

/**
 * @brief Set the formation_handled_ object to False.
 */
void Pelican::unsetAndNotifyFormationHandled() {
    std::lock_guard rd_lock(this->form_tristate_mutex_);
    this->formation_handled_ = TriState::False;
    this->formation_handled_cv_.notify_all();
}

/**
 * @brief Set the formation_achieved_ flag to True.
 */
void Pelican::setFormationAchieved() {
    std::lock_guard rd_lock(this->form_achieved_mutex_);
    this->formation_achieved_ = true;
}

/**
 * @brief Set the formation_achieved_ flag to False.
 */
void Pelican::unsetFormationAchieved() {
    std::lock_guard rd_lock(this->form_achieved_mutex_);
    this->formation_achieved_ = false;
}
