#ifndef __TACMAP_HPP__
#define __TACMAP_HPP__

#include "LoggerModule/logger.hpp"
#include "types.hpp"
#include "utilities.hpp"

using std::literals::string_literals::operator""s;

class Pelican;

class TacMapModule {
    public:
        // Ctors/Dctors
        explicit TacMapModule();
        explicit TacMapModule(Pelican*);
        ~TacMapModule();

        // Setup and behavior methods
        void initSetup(LoggerModule*);
        void stopService();

        // Transmitting data
        void publishOffboardControlMode();
        void publishTrajectorySetpoint();
        void publishVehicleCommand(
            uint16_t, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN,
            float = NAN
        );

        std::optional<px4_msgs::msg::VehicleGlobalPosition> getGlobalPosition();
        std::optional<px4_msgs::msg::VehicleOdometry> getOdometry();
        std::optional<px4_msgs::msg::VehicleCommandAck> getAck();
        std::optional<px4_msgs::msg::VehicleStatus> getStatus();

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

        // Topics-related setups
        void initTopics();
        void initSubscribers();
        void initPublishers();

        // Receiving data
        void printData(const px4_msgs::msg::FailsafeFlags::SharedPtr) const;
        void printData(const px4_msgs::msg::VehicleAttitude::SharedPtr) const;
        void printData(const px4_msgs::msg::VehicleControlMode::SharedPtr) const;
        void storeGlobalPosition(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr);
        void printData(const px4_msgs::msg::VehicleLocalPosition::SharedPtr) const;
        void storeOdometry(const px4_msgs::msg::VehicleOdometry::SharedPtr);
        void storeStatus(const px4_msgs::msg::VehicleStatus::SharedPtr);
        void storeAck(const px4_msgs::msg::VehicleCommandAck::SharedPtr);

        bool checkIsRunning();

    private: // Attributes
        Pelican* node_;
        LoggerModule* logger_;

        bool running_ {true};
        int standard_qos_value_ = 10;

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

        // CHECK: needed?
        rclcpp::TimerBase::SharedPtr offboard_timer_;
        std::chrono::milliseconds offboard_period_ {100};
        uint64_t offboard_setpoint_counter_; // counter for the number of setpoints sent

        std::string flags_topic_;            // i.e. "/px4_{id}/fmu/out/failsafe_flags";
        rclcpp::Subscription<px4_msgs::msg::FailsafeFlags>::SharedPtr sub_to_flags_topic_;

        std::string attitude_topic_; // i.e. "/px4_{id}/fmu/out/vehicle_attitude";
        rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr sub_to_attitude_topic_;

        std::string control_mode_topic_; // i.e. "/px4_{id}/fmu/out/vehicle_control_mode";
        rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr
            sub_to_control_mode_topic_;

        std::string global_pos_topic_; // i.e. "/px4_{id}/fmu/out/vehicle_global_position";
        rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr
            sub_to_global_pos_topic_;

        std::string local_pos_topic_; // i.e. "/px4_{id}/fmu/out/vehicle_local_position";
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
            sub_to_local_pos_topic_;

        std::string odometry_topic_; // i.e. "/px4_{id}/fmu/out/vehicle_odometry";
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_to_odometry_topic_;

        std::string status_topic_; // i.e. "/px4_{id}/fmu/out/vehicle_status";
        rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_to_status_topic_;

        std::string command_ack_topic_; // i.e. "/px4_{id}/fmu/out/vehicle_command_ack";
        rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr sub_to_command_ack_topic_;

        std::string command_topic_; // i.e. "/px4_{id}/fmu/in/vehicle_command";
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_to_command_topic_;

        std::string offboard_control_topic_; // i.e. "/px4_{id}/fmu/in/offboard_control_mode";
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
            pub_to_offboard_control_topic_;

        std::string trajectory_setpoint_topic_; // i.e. "/px4_{id}/fmu/in/trajectory_setpoint";
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
            pub_to_trajectory_setpoint_topic;

        // Only one ack memorized because messages from that topic should be sparse
        std::optional<px4_msgs::msg::VehicleCommandAck> last_ack_;
        boost::circular_buffer<px4_msgs::msg::VehicleGlobalPosition> globalpos_buffer_ {10};
        boost::circular_buffer<px4_msgs::msg::VehicleOdometry> odometry_buffer_ {10};
        boost::circular_buffer<px4_msgs::msg::VehicleStatus> status_buffer_ {10};

        mutable std::mutex running_mutex_;   // to be used with running_
        mutable std::mutex ack_mutex_;       // to be used with last_ack_
        mutable std::mutex globalpos_mutex_; // to be used with gps_buffer_
        mutable std::mutex odometry_mutex_;  // to be used with odometry_buffer_
        mutable std::mutex status_mutex_;    // to be used with status_buffer_
};

#include "tacmap_template.tpp"

#endif
