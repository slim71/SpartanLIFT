#include "datapad.hpp"

std::shared_ptr<Datapad> Datapad::getInstance() {
    return instance_.lock();
}

bool Datapad::isRunning() const {
    std::lock_guard lock(this->running_mutex_);
    return this->running_;
}

TriState Datapad::isCargoHandled() const {
    std::lock_guard lock(this->cargo_tristate_mutex_);
    return this->cargo_handled_;
}
