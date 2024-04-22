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
        bool returnToLaunchPosition();
        void activateOffboardMode();

        // Getters
        bool getRunningStatus() const;
        Eigen::Vector3d getOffset() const;

    private:
        template<typename... Args> void sendLogInfo(std::string, Args...) const;
        template<typename... Args> void sendLogDebug(std::string, Args...) const;
        template<typename... Args> void sendLogWarning(std::string, Args...) const;
        template<typename... Args> void sendLogError(std::string, Args...) const;

        void runPreChecks();
        void setAndMaintainOffboardMode();
        void consensusToRendezvous();
        geometry_msgs::msg::Point adjustmentForCollisionAvoidance(geometry_msgs::msg::Point);

        bool sendToCommanderUnit(
            uint16_t, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN,
            float = NAN
        );

        // External communications
        rclcpp::Time gatherTime() const;
        unsigned int gatherAgentID() const;
        double gatherROI() const;
        double gatherCollisionRadius() const;
        std::optional<px4_msgs::msg::VehicleOdometry> gatherOdometry() const;
        std::optional<px4_msgs::msg::VehicleStatus> gatherStatus() const;
        std::optional<geometry_msgs::msg::Point> gatherTargetPosition() const;
        std::optional<geometry_msgs::msg::Point> gatherSetpointVelocity() const;
        std::optional<geometry_msgs::msg::Point> gatherDesiredPosition() const;
        unsigned int gatherNetworkSize() const;
        geometry_msgs::msg::Point gatherCopterPosition(unsigned int) const;
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
        void signalSetSetpointPosition(geometry_msgs::msg::Point) const;
        void signalSetSetpointVelocity(geometry_msgs::msg::Point) const;

    private: // Attributes
        Pelican* node_;
        LoggerModule* logger_;

        // For possible future use
        Eigen::Vector3d offset_ {0, 0, 0}; // [m, m, m]
        std::atomic<bool> running_ {true};
        bool sitl_ready_ {false};
        uint64_t offboard_setpoint_counter_ {0}; // counter for the number of setpoints sent

        mutable std::mutex offset_mutex_;        // to be used with offset_ and yaw_
        mutable std::mutex running_mutex_;       // to be used with running_

        rclcpp::TimerBase::SharedPtr prechecks_timer_;
        std::chrono::seconds prechecks_period_ {constants::PRECHECKS_TIME_SECS};

        rclcpp::TimerBase::SharedPtr offboard_timer_;
        std::chrono::milliseconds offboard_period_ {constants::OFFBOARD_PERIOD_MILLIS};

        rclcpp::TimerBase::SharedPtr rendezvous_timer_;
        std::chrono::milliseconds rendezvous_period_ {
            std::chrono::milliseconds(constants::RENDEZVOUS_CONSENSUS_PERIOD_MILLIS)};
};

#include "unsc_template.tpp"

#endif
