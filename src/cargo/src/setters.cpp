/**
 * @file setters.cpp
 * @author Simone Vollaro (slim71sv@gmail.com)
 * @brief File containing setter methods for the Cargo class.
 * @version 1.0.0
 * @date 2024-11-13
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "cargo.hpp"

/**
 * @brief Set the running_ flag to True.
 *
 */
void Cargo::setRunning() {
    std::lock_guard lock(this->running_mutex_);
    this->running_ = true;
}

/**
 * @brief Set the running_ flag to False.
 *
 */
void Cargo::unsetRunning() {
    std::lock_guard lock(this->running_mutex_);
    this->running_ = false;
}
