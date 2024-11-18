/**
 * @file getters.cpp
 * @author Simone Vollaro (slim71sv@gmail.com)
 * @brief File containing getter methods for the Datapad class.
 * @version 1.0.0
 * @date 2024-11-13
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "datapad.hpp"

/**
 * @brief Retrieve the instance linked to the Cargo object.
 *
 * @return std::shared_ptr<Cargo> The instance itself.
 */
std::shared_ptr<Datapad> Datapad::getInstance() {
    return instance_.lock();
}

/**
 * @brief Return the status of the running_ flag.
 *
 * @return true
 * @return false
 */
bool Datapad::isRunning() const {
    std::lock_guard lock(this->running_mutex_);
    return this->running_;
}

/**
 * @brief Return the status of the cargo_handled_ flag.
 *
 * @return true
 * @return false
 */
TriState Datapad::isCargoHandled() const {
    std::lock_guard lock(this->cargo_tristate_mutex_);
    return this->cargo_handled_;
}
