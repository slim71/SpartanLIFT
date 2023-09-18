#include "PelicanModule/pelican.hpp"
#include "LoggerModule/logger.hpp"
#include "HeartbeatModule/heartbeat.hpp"
#include "ElectionModule/election.hpp"
#include "types.hpp"
#include "gtest/gtest.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

class PelicanTest : public ::testing::Test {
    protected: // Member functions
        void SetUp() override;
        void TearDown() override;

        void PositionPublisherTester();
        void HeartbeatPublisherTester();
        void DatapadPublisherTester();
        int RequestNumberOfHbsTester();
        heartbeat RequestLastHbTester();
        void CommenceFollowerOperationsTester();
        void CommenceCandidateOperationsTester();
        void CommenceLeaderOperationsTester();

    protected: // Attributes
        std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
        std::shared_ptr<Pelican> node_;

        std::thread spin_thread_;

        mutable std::mutex data_ok_mutex_;

        bool data_ok_ {false};

        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;
        rclcpp::Subscription<comms::msg::Heartbeat>::SharedPtr hb_sub_;
        rclcpp::Subscription<comms::msg::Datapad>::SharedPtr data_sub_;

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
};

class HeartbeatTest : public ::testing::Test {
    protected:
        HeartbeatModule core_;
};

class ElectionTest : public ::testing::Test {
    protected:
        ElectionModule core_;
};

class LoggerTest : public ::testing::Test {
    protected: // Member functions
        void SetUp() override;

    protected: // Attributes
        std::shared_ptr<rclcpp::Logger> l_;
        std::shared_ptr<LoggerModule> core_;
};
