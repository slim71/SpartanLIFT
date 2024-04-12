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
        Eigen::Vector3f getOffset() const;

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

        Eigen::Vector3f
        convertLocalToBody(const Eigen::Vector3f&) const; // CHECK: just a util func?

        // External communications
        rclcpp::Time gatherTime() const;
        unsigned int gatherAgentID() const;
        rclcpp::CallbackGroup::SharedPtr gatherReentrantGroup() const;
        std::optional<px4_msgs::msg::VehicleGlobalPosition> gatherGlobalPosition() const;
        std::optional<px4_msgs::msg::VehicleOdometry> gatherOdometry() const;
        std::optional<px4_msgs::msg::VehicleStatus> gatherStatus() const;
        std::optional<std::vector<float>> gatherTargetPose() const;
        std::optional<std::vector<float>> gatherTargetVelocity() const;
        std::optional<std::vector<float>> gatherDesiredPose() const;
        unsigned int gatherNetworkSize() const;
        geometry_msgs::msg::Point gatherCopterPosition(unsigned int);

        // command| param1| param2| param3| param4| param5| param6| param7|
        void signalPublishVehicleCommand(
            uint16_t, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN,
            float = NAN
        ) const;
        void signalPublishTrajectorySetpoint(float, float, float, float, float, float) const;
        void signalPublishOffboardControlMode() const;
        bool signalWaitForCommanderAck(uint16_t) const;
        bool signalCheckOffboardEngagement() const;
        void signalSetSetpointPosition(float, float, float = std::nan("")) const;
        void signalSetTargetVelocity(float, float) const;

    private: // Attributes
        Pelican* node_;
        LoggerModule* logger_;

        // For possible future use
        Eigen::Vector3f offset_ {0, 0, 0}; // [m, m, m]
        float yaw_ {0};                    // [rad]
        mutable std::mutex offset_mutex_;  // to be used with offset_ and yaw_

        std::atomic<bool> running_ {true};
        bool sitl_ready_ {false};
        mutable std::mutex running_mutex_; // to be used with running_

        rclcpp::TimerBase::SharedPtr starting_timer_;
        std::chrono::seconds briefing_time_ {constants::BRIEFING_TIME_SECS}; // TODO: change names?

        rclcpp::TimerBase::SharedPtr offboard_timer_;
        std::chrono::milliseconds offboard_period_ {constants::OFFBOARD_PERIOD_MILLIS};
        uint64_t offboard_setpoint_counter_ {0}; // counter for the number of setpoints sent

        rclcpp::TimerBase::SharedPtr rendezvous_timer_;
        std::chrono::milliseconds rendezvous_period_ {
            std::chrono::milliseconds(constants::RENDEZVOUS_CONSENSUS_PERIOD_MILLIS)};
};

#include "unsc_template.tpp"

#endif
