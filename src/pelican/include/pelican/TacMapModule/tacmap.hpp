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
        void publishTrajectorySetpoint(geometry_msgs::msg::Point, geometry_msgs::msg::Point);
        void publishOffboardControlMode();
        void publishVehicleCommand(
            uint16_t, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN,
            float = NAN
        );
        bool waitForCommanderAck(uint16_t);

        std::optional<nav_msgs::msg::Odometry> getENUOdometry() const;
        std::optional<px4_msgs::msg::VehicleCommandAck> getCommanderAck() const;
        std::optional<px4_msgs::msg::VehicleStatus> getStatus() const;
        bool getRunningStatus() const;
        bool getInitiatedStatus() const;

        void setInitiatedStatus(bool);

        bool checkOffboardEngagement();

    private:
        template<typename... Args> void sendLogInfo(std::string, Args...) const;
        template<typename... Args> void sendLogDebug(std::string, Args...) const;
        template<typename... Args> void sendLogWarning(std::string, Args...) const;
        template<typename... Args> void sendLogError(std::string, Args...) const;

        // Topics-related setups
        void initTopics();
        void initSubscribers();
        void initPublishers();

        void triggerHeightTracking();

        // Receiving data
        void storeStatus(const px4_msgs::msg::VehicleStatus::SharedPtr);
        void storeAck(const px4_msgs::msg::VehicleCommandAck::SharedPtr);
        void storeENUOdometry(const nav_msgs::msg::Odometry::SharedPtr);

        // External communications
        unsigned int gatherAgentID() const;
        std::string gatherAgentModel() const;
        possible_roles gatherAgentRole() const;
        int gatherCurrentTerm() const;
        rclcpp::Time gatherTime() const;
        rclcpp::CallbackGroup::SharedPtr gatherReentrantGroup() const;
        rclcpp::SubscriptionOptions gatherReentrantOptions() const;
        void signalHeightCompensation(double) const;
        void signalSharePosition(geometry_msgs::msg::Point);

    private: // Attributes
        Pelican* node_;
        LoggerModule* logger_;
        bool running_ {true};
        int system_id_ {0};
        int component_id_ {0};

        // Only one ack memorized because messages from that topic should be sparse
        std::optional<px4_msgs::msg::VehicleCommandAck> last_commander_ack_;

        boost::circular_buffer<nav_msgs::msg::Odometry> enu_odometry_buffer_ {
            constants::ENU_ODOMETRY_BUFFER_SIZE};
        boost::circular_buffer<px4_msgs::msg::VehicleStatus> status_buffer_ {
            constants::STATUS_BUFFER_SIZE};

        mutable std::mutex running_mutex_;       // to be used with running_
        mutable std::mutex commander_ack_mutex_; // to be used with last_commander_ack_
        mutable std::mutex globalpos_mutex_;     // to be used with gps_buffer_
        mutable std::mutex enu_odometry_mutex_;  // to be used with enu_odometry_buffer_
        mutable std::mutex status_mutex_;        // to be used with status_buffer_
        mutable std::mutex system_id_mutex_;     // to be used with system_id_
        mutable std::mutex component_id_mutex_;  // to be used with component_id_

        // The subscription sets a QoS profile based on rmw_qos_profile_sensor_data.
        // This is needed because the default ROS 2 QoS profile for subscribers is
        // incompatible with the PX4 profile for publishers.
        rclcpp::QoS px4_qos_ {rclcpp::QoS(
            rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, 5),
            rmw_qos_profile_sensor_data
        )};
        rclcpp::QoS standard_qos_ {rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default), rmw_qos_profile_default
        )};

        std::string flags_topic_; // i.e. "/px4_{id}/fmu/out/failsafe_flags";
        rclcpp::Subscription<px4_msgs::msg::FailsafeFlags>::SharedPtr sub_to_flags_topic_;

        std::string attitude_topic_; // i.e. "/px4_{id}/fmu/out/vehicle_attitude";
        rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr sub_to_attitude_topic_;

        std::string control_mode_topic_; // i.e. "/px4_{id}/fmu/out/vehicle_control_mode";
        rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr
            sub_to_control_mode_topic_;

        std::string status_topic_; // i.e. "/px4_{id}/fmu/out/vehicle_status";
        rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_to_status_topic_;

        std::string command_ack_topic_; // i.e. "/px4_{id}/fmu/out/vehicle_command_ack";
        rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr sub_to_command_ack_topic_;

        std::string command_topic_; // i.e. "/px4_{id}/fmu/in/vehicle_command";
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_to_command_topic_;

        std::string trajectory_setpoint_topic_; // i.e. "/px4_{id}/fmu/in/trajectory_setpoint";
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
            pub_to_trajectory_setpoint_topic_;

        std::string offboard_control_topic_; // i.e. "/px4_{id}/fmu/in/offboard_control_mode";
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
            pub_to_offboard_control_topic_;

        std::string enu_odometry_topic_; // i.e. "model/{model_name}_{agent_id}/odometry";
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_to_enu_odometry_topic_;

        rclcpp::TimerBase::SharedPtr position_timer_;
        rclcpp::TimerBase::SharedPtr compensation_timer_;
};

#include "tacmap_template.tpp"

#endif
