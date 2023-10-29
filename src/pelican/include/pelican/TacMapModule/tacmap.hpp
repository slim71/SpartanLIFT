#ifndef __TACMAP_HPP__
#define __TACMAP_HPP__

#include "LoggerModule/logger.hpp"
#include "px4_msgs/msg/failsafe_flags.hpp"
#include "px4_msgs/msg/sensor_gps.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_control_mode.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "types.hpp"
#include "utilities.hpp"
#include <iostream>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>

using std::literals::string_literals::operator""s;

class Pelican;

class TacMapModule {
    public:
        // Ctors/Dctors
        explicit TacMapModule();
        explicit TacMapModule(Pelican*);
        ~TacMapModule();

        // Setup methods
        void initSetup(LoggerModule*);

    private:
        template<typename... Args> void sendLogInfo(std::string, Args...) const;
        template<typename... Args> void sendLogDebug(std::string, Args...) const;
        template<typename... Args> void sendLogWarning(std::string, Args...) const;
        template<typename... Args> void sendLogError(std::string, Args...) const;

        // External communications
        int gatherAgentID() const;
        possible_roles gatherAgentRole() const;
        int gatherCurrentTerm() const;
        rclcpp::Time gatherTime() const;
        rclcpp::CallbackGroup::SharedPtr gatherReentrantGroup() const;
        rclcpp::SubscriptionOptions gatherReentrantOptions() const;

        // Topics-related
        void initTopics();
        void initSubscribers();
        void initPublishers();

        // TODO: change name
        void printData(const px4_msgs::msg::FailsafeFlags::SharedPtr) const;
        void printData(const px4_msgs::msg::VehicleAttitude::SharedPtr) const;
        void printData(const px4_msgs::msg::VehicleControlMode::SharedPtr) const;
        void printData(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr) const;
        void printData(const px4_msgs::msg::SensorGps::SharedPtr) const;
        void printData(const px4_msgs::msg::VehicleLocalPosition::SharedPtr) const;
        void printData(const px4_msgs::msg::VehicleOdometry::SharedPtr) const;
        void printData(const px4_msgs::msg::VehicleStatus::SharedPtr) const;

        void offboardTimerCallback();

        void arm();
        void disarm();
        void publishVehicleCommand(uint16_t, float = 0.0, float = 0.0);
        void publishOffboardControlMode();
        void publishTrajectorySetpoint();
        void takeoffAttempt();

    private: // Attributes
        Pelican* node_;
        LoggerModule* logger_;

        // The subscription sets a QoS profile based on rmw_qos_profile_sensor_data.
        // This is needed because the default ROS 2 QoS profile for subscribers is
        // incompatible with the PX4 profile for publishers.
        rclcpp::QoS px4_qos_ {rclcpp::QoS(
            rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, 5),
            rmw_qos_profile_sensor_data
        )};
        rmw_qos_profile_t qos_profile_ {rmw_qos_profile_default};
        rclcpp::QoS standard_qos_ {
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile_), qos_profile_)};

        rclcpp::TimerBase::SharedPtr offboard_timer_;
        uint64_t offboard_setpoint_counter_; // counter for the number of setpoints sent

        // CHECK: maybe
        std::string flags_topic_; // i.e. "/px4_{id}/fmu/out/failsafe_flags";
        rclcpp::Subscription<px4_msgs::msg::FailsafeFlags>::SharedPtr sub_to_flags_topic_;

        // CHECK: maybe
        std::string attitude_topic_; // i.e. "/px4_{id}/fmu/out/vehicle_attitude";
        rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr sub_to_attitude_topic_;

        std::string control_mode_topic_; // i.e. "/px4_{id}/fmu/out/vehicle_control_mode";
        rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr
            sub_to_control_mode_topic_;

        std::string global_pos_topic_; // i.e. "/px4_{id}/fmu/out/vehicle_global_position";
        rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr
            sub_to_global_pos_topic_;

        // CHECK: maybe
        std::string gps_pos_topic_; // i.e. "/px4_{id}/fmu/out/vehicle_gps_position";
        rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr sub_to_gps_pos_topic_;

        std::string local_pos_topic_; // i.e. "/px4_{id}/fmu/out/vehicle_local_position";
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
            sub_to_local_pos_topic_;

        std::string odometry_topic_; // i.e. "/px4_{id}/fmu/out/vehicle_odometry";
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_to_odometry_topic_;

        std::string status_topic_; // i.e. "/px4_{id}/fmu/out/vehicle_status";
        rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_to_status_topic_;

        // TODO: needs publishers for waypoints/trajectory too?

        // CHECK: there's an ack?
        std::string command_topic_; // i.e. "/px4_{id}/fmu/in/vehicle_command";
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_to_command_topic_;

        std::string offboard_control_topic_; // i.e. "/px4_{id}/fmu/in/offboard_control_mode";
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
            pub_to_offboard_control_topic_;

        std::string trajectory_setpoint_topic_; // i.e. "/px4_{id}/fmu/in/trajectory_setpoint";
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
            pub_to_trajectory_setpoint_topic;
};

#include "tacmap_template.tpp"

#endif
