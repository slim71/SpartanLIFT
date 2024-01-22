#include "ElectionModule/election.hpp"
#include "HeartbeatModule/heartbeat.hpp"
#include "LoggerModule/logger.hpp"
#include "PelicanModule/pelican.hpp"
#include "TacMapModule/tacmap.hpp"
#include "UNSCModule/unsc.hpp"
#include "types.hpp"
#include "gtest/gtest.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>

class PelicanTest : public ::testing::Test {
    protected: // Member functions
        void SetUp() override;
        void TearDown() override;

        void HeartbeatPublisherTester();
        void ProposalPublisherTester();
        int RequestNumberOfHbsTester();
        heartbeat RequestLastHbTester();
        std::optional<px4_msgs::msg::VehicleGlobalPosition> RequestGlobalPositionTester();
        std::optional<px4_msgs::msg::VehicleOdometry> RequestOdometryTester();
        std::optional<px4_msgs::msg::VehicleCommandAck> RequestAckTester();
        std::optional<px4_msgs::msg::VehicleStatus> RequestStatusTester();
        void CommenceFollowerOperationsTester();
        void CommenceCandidateOperationsTester();
        void CommenceLeaderOperationsTester();
        void CommencePublishVehicleCommandTester();

    protected: // Attributes
        std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
        std::shared_ptr<Pelican> node_;

        std::thread spin_thread_;

        mutable std::mutex data_ok_mutex_;
        bool data_ok_ {false};

        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;
        rclcpp::Subscription<comms::msg::Heartbeat>::SharedPtr hb_sub_;
        rclcpp::Subscription<comms::msg::Proposal>::SharedPtr data_sub_;
        rclcpp::Subscription<px4_msgs::msg::VehicleCommand>::SharedPtr command_sub_;

        rclcpp::QoS px4_qos_ {rclcpp::QoS(
            rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, 5),
            rmw_qos_profile_sensor_data
        )};
        rclcpp::QoS standard_qos_ {rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default), rmw_qos_profile_default
        )};

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

class TacMapTest : public ::testing::Test {
    protected: // Member functions
        void SetUp() override;
        void TearDown() override;

        void VehicleCommandPublisherTester();

    protected: // Attributes
        TacMapModule core_;
        std::shared_ptr<rclcpp::Node> node_;

        mutable std::mutex data_ok_mutex_;
        bool data_ok_ {false};

        rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_sub_;
        rclcpp::Subscription<px4_msgs::msg::VehicleCommand>::SharedPtr command_sub_;

        rclcpp::QoS px4_qos_ {rclcpp::QoS(
            rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, 5),
            rmw_qos_profile_sensor_data
        )};

        rclcpp::CallbackGroup::SharedPtr reentrant_group_;
        rclcpp::SubscriptionOptions reentrant_opt_ {rclcpp::SubscriptionOptions()};
};

class UNSCTest : public ::testing::Test {
    protected:
        UNSCModule core_;
};

class LoggerTest : public ::testing::Test {
    protected: // Member functions
        void SetUp() override;

    protected: // Attributes
        std::shared_ptr<rclcpp::Logger> l_;
        std::shared_ptr<LoggerModule> core_;
};
