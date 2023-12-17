#ifndef __UNSC_HPP__
#define __UNSC_HPP__

#include "LoggerModule/logger.hpp"
// #include <mavsdk/mavsdk.h>
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

        bool getRunningStatus();

    private:
        template<typename... Args> void sendLogInfo(std::string, Args...) const;
        template<typename... Args> void sendLogDebug(std::string, Args...) const;
        template<typename... Args> void sendLogWarning(std::string, Args...) const;
        template<typename... Args> void sendLogError(std::string, Args...) const;

        // External communications
        rclcpp::Time gatherTime() const;
        rclcpp::CallbackGroup::SharedPtr gatherReentrantGroup() const;
        std::optional<px4_msgs::msg::VehicleGlobalPosition> gatherGlobalPosition() const;
        std::optional<px4_msgs::msg::VehicleOdometry> gatherOdometry() const;
        std::optional<px4_msgs::msg::VehicleCommandAck> gatherAck() const;
        std::optional<px4_msgs::msg::VehicleStatus> gatherStatus() const;

        void arm();
        void disarm();

        void takeoff(unsigned int = 0);
        void land();
        void setHome();
        void setPositionMode();
        void returnToLaunchPosition();
        void lowerLoiter(); // TODO:?

        void runPreChecks();
        void setAndMaintainOffboardMode(float, float, float, float);

        bool waitForAck(uint16_t); // TODO: do after each command?

        // command| param1| param2| param3| param4| param5| param6| param7|
        void signalPublishVehicleCommand(
            uint16_t, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN,
            float = NAN
        ) const;
        void signalPublishTrajectorySetpoint(float, float, float, float) const;
        void signalPublishOffboardControlMode() const;

    private: // Attributes
        Pelican* node_;
        LoggerModule* logger_;

        std::atomic<bool> running_ {true};
        bool sitl_ready_ {false};
        mutable std::mutex running_mutex_; // to be used with running_

        rclcpp::TimerBase::SharedPtr starting_timer_;
        std::chrono::seconds briefing_time_ {10};

        rclcpp::TimerBase::SharedPtr offboard_timer_;
        std::chrono::milliseconds offboard_period_ {100};
        uint64_t offboard_setpoint_counter_ {0}; // counter for the number of setpoints sent
};

#include "unsc_template.tpp"

#endif
