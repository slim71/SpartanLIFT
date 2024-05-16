#include "datapad.hpp"

std::shared_ptr<Datapad> Datapad::getInstance() {
    return instance_.lock();
}

bool Datapad::isRunning() const {
    std::lock_guard lock(this->running_mutex_);
    return this->running_;
}
