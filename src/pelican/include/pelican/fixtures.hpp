#include "gtest/gtest.h"
#include "pelican.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

class PelicanTest : public ::testing::Test {
    protected:
        std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
        std::shared_ptr<Pelican> node;
        std::thread spin_thread;
        mutable std::mutex data_ok_mutex;
        bool data_ok {false};
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub;
        rclcpp::Subscription<comms::msg::Heartbeat>::SharedPtr hb_sub;
        rclcpp::Subscription<comms::msg::Datapad>::SharedPtr data_sub;
        rclcpp::QoS px4_qos_ {rclcpp::QoS(
                rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, 5),
                rmw_qos_profile_sensor_data
            )};
        rmw_qos_profile_t qos_profile_ {rmw_qos_profile_default};
        rclcpp::QoS standard_qos_ {
                rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile_), qos_profile_)
            };
        rclcpp::CallbackGroup::SharedPtr reentrant_group_;
        rclcpp::SubscriptionOptions reentrant_opt_ {rclcpp::SubscriptionOptions()};

        void SetUp() override;
        void TearDown() override;

        // void TestPositionSubscription();
        // void TestSubscription();
        // void TestSubscription();
        void PositionPublisherTester();
        void HeartbeatPublisherTester();
        void DatapadPublisherTester();
};
