#include "datapad.hpp"

void Datapad::setInstance(rclcpp::Node::SharedPtr instance) {
    instance_ = std::static_pointer_cast<Datapad>(instance);
}
