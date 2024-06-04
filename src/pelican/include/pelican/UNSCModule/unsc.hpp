#ifndef __UNSC_HPP__
#define __UNSC_HPP__

#include "LoggerModule/logger.hpp"
#include "types.hpp"
#include "utilities.hpp"

class Pelican;

class UNSCModule {
    public:
        // Ctors/Dctors
        explicit UNSCModule();
        explicit UNSCModule(Pelican*);
        ~UNSCModule();

        // Setup methods
        void initSetup(LoggerModule*);
        void stopService();

        bool arm();
        bool disarm();
        bool takeoff(unsigned int = 0);
        bool land();
        bool setHome();
        bool loiter();
        bool returnToLaunchPosition();
        void activateOffboardMode();
        void heightCompensation(double);

        // Getters
        bool getRunningStatus() const;
        Eigen::Vector3d getOffset() const;
        std::optional<geometry_msgs::msg::Point> getSetpointPosition() const;
        std::optional<geometry_msgs::msg::Point> getSetpointVelocity() const;
        double getActualTargetHeight() const;
        std::optional<geometry_msgs::msg::Point> getTargetPosition() const;

        // Setters
        void setSetpointPosition(geometry_msgs::msg::Point);
        void setSetpointVelocity(geometry_msgs::msg::Point);
        void setActualTargetHeight(double);
        void setTargetPosition(geometry_msgs::msg::Point);

    private:
        template<typename... Args> void sendLogInfo(std::string, Args...) const;
        template<typename... Args> void sendLogDebug(std::string, Args...) const;
        template<typename... Args> void sendLogWarning(std::string, Args...) const;
        template<typename... Args> void sendLogError(std::string, Args...) const;

        void runPreChecks();
        void setAndMaintainOffboardMode();
        geometry_msgs::msg::Point adjustmentForCollisionAvoidance(geometry_msgs::msg::Point);
        void consensusToRendezvous();
        void rendezvousClosure();

        bool sendToCommanderUnit(
            uint16_t, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN,
            float = NAN
        );

        // External communications
        rclcpp::Time gatherTime() const;
        unsigned int gatherAgentID() const;
        double gatherROI() const;
        double gatherCollisionRadius() const;
        std::optional<px4_msgs::msg::VehicleStatus> gatherStatus() const;
        unsigned int gatherNetworkSize() const;
        geometry_msgs::msg::Point gatherCopterPosition(unsigned int) const;
        std::optional<nav_msgs::msg::Odometry> gatherENUOdometry() const;

        // For callback groups
        rclcpp::CallbackGroup::SharedPtr gatherReentrantGroup() const;
        rclcpp::CallbackGroup::SharedPtr gatherOffboardExclusiveGroup() const;
        rclcpp::CallbackGroup::SharedPtr gatherRendezvousExclusiveGroup() const;

        // command| param1| param2| param3| param4| param5| param6| param7|
        void signalPublishVehicleCommand(
            uint16_t, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN,
            float = NAN
        ) const;
        void signalPublishTrajectorySetpoint(geometry_msgs::msg::Point, geometry_msgs::msg::Point)
            const;
        void signalPublishOffboardControlMode() const;
        bool signalWaitForCommanderAck(uint16_t) const;
        bool signalCheckOffboardEngagement() const;
        void signalSetReferenceHeight(double);
        void signalCargoAttachment();

        // Flag checks
        bool confirmAgentIsLeader() const;

    private: // Attributes
        Pelican* node_;
        LoggerModule* logger_;

        // For possible future use
        bool running_ {true};
        bool sitl_ready_ {false};
        bool move_to_center_ {false};
        uint64_t offboard_setpoint_counter_ {0}; // counter for the number of setpoints sent
        double actual_target_height_ {0};        // needed in order to stabilize height tracking
        Eigen::Vector3d offset_ {0, 0, 0};       // [m, m, m]
        geometry_msgs::msg::Point target_position_ = NAN_point; // Actual desired target
        // referring to temporary setpoint along a trajectory
        geometry_msgs::msg::Point setpoint_position_ = NAN_point;
        // referring to temporary setpoint along a trajectory
        geometry_msgs::msg::Point setpoint_velocity_;

        mutable std::mutex offset_mutex_;            // to be used with offset_ and yaw_
        mutable std::mutex running_mutex_;           // to be used with running_
        mutable std::mutex setpoint_position_mutex_; // Used to access setpoint_position_
        mutable std::mutex setpoint_velocity_mutex_; // Used to access setpoint_velocity_
        mutable std::mutex height_mutex_;            // Used to access actual_target_height_
        mutable std::mutex target_position_mutex_;   // Used to access target_position_

        rclcpp::TimerBase::SharedPtr prechecks_timer_;
        std::chrono::seconds prechecks_period_ {constants::PRECHECKS_TIME_SECS};

        rclcpp::TimerBase::SharedPtr offboard_timer_;
        std::chrono::milliseconds offboard_period_ {constants::OFFBOARD_PERIOD_MILLIS};

        rclcpp::TimerBase::SharedPtr rendezvous_timer_;
        std::chrono::milliseconds rendezvous_period_ {
            std::chrono::milliseconds(constants::RENDEZVOUS_CONSENSUS_PERIOD_MILLIS)};

        rclcpp::TimerBase::SharedPtr rend_check_timer_;
        std::chrono::milliseconds rend_check_period_ {std::chrono::seconds(1)};
};

#include "unsc_template.tpp"

#endif
