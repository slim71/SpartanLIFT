#include "cargo.hpp"

void Cargo::setRunning() {
    std::lock_guard lock(this->running_mutex_);
    this->running_ = true;
}

void Cargo::unsetRunning() {
    std::lock_guard lock(this->running_mutex_);
    this->running_ = false;
}
