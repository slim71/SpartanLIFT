#include "datapad.hpp"
#include "types.hpp"
#include "gtest/gtest.h"
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

class DatapadTest : public ::testing::Test {
    protected: // Member functions
        void SetUp() override;
        void TearDown() override;

    protected: // Attributes
        std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
        std::shared_ptr<Datapad> node_;

        std::thread spin_thread_;

        mutable std::mutex data_ok_mutex_;
        bool data_ok_ {false};

        rclcpp::QoS standard_qos_ {rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default), rmw_qos_profile_default
        )};

        rclcpp::CallbackGroup::SharedPtr reentrant_group_;
        rclcpp::SubscriptionOptions reentrant_opt_ {rclcpp::SubscriptionOptions()};

        rclcpp::Subscription<comms::msg::POI>::SharedPtr sub_to_channel;
};
