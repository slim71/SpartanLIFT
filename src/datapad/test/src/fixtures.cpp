#include "fixtures.hpp"

void DatapadTest::SetUp() {
    // Initialization
    rclcpp::init(0, nullptr);

    // Instantiation
    this->node_ = std::make_shared<Datapad>();
    // Set the instance pointer to the shared pointer of the main node
    Datapad::setInstance(this->node_);

    this->reentrant_group_ =
        this->node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    this->reentrant_opt_.callback_group = this->reentrant_group_;

    this->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    this->executor_->add_node(this->node_);

    this->spin_thread_ = std::thread([this]() {
        this->executor_->spin();
    });
}

void DatapadTest::TearDown() {
    rclcpp::shutdown();

    this->spin_thread_.join(); // Wait for thread completion
}
