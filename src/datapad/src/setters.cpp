#include "datapad.hpp"

void Datapad::setInstance(rclcpp::Node::SharedPtr instance) {
    instance_ = std::static_pointer_cast<Datapad>(instance);
}

void Datapad::setAndNotifyCargoHandled() {
    std::lock_guard lock(this->cargo_tristate_mutex_);
    this->cargo_handled_ = TriState::True;
    this->cargo_cv_.notify_all();
}

void Datapad::unsetAndNotifyCargoHandled() {
    std::lock_guard lock(this->cargo_tristate_mutex_);
    this->cargo_handled_ = TriState::False;
    this->cargo_cv_.notify_all();
}
