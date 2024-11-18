/**
 * @file setters.cpp
 * @author Simone Vollaro (slim71sv@gmail.com)
 * @brief File containing setter methods for the Datapad class.
 * @version 1.0.0
 * @date 2024-11-13
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "datapad.hpp"

/**
 * @brief Set the instance linked to the Datapad object.
 *
 * @param instance Datapad instance to link.
 */
void Datapad::setInstance(rclcpp::Node::SharedPtr instance) {
    instance_ = std::static_pointer_cast<Datapad>(instance);
}

/**
 * @brief Set the cargo_handled_ structure to True.
 *
 */
void Datapad::setAndNotifyCargoHandled() {
    std::lock_guard lock(this->cargo_tristate_mutex_);
    this->cargo_handled_ = TriState::True;
    this->cargo_cv_.notify_all();
}

/**
 * @brief Set the cargo_handled_ structure to False.
 *
 */
void Datapad::unsetAndNotifyCargoHandled() {
    std::lock_guard lock(this->cargo_tristate_mutex_);
    this->cargo_handled_ = TriState::False;
    this->cargo_cv_.notify_all();
}
