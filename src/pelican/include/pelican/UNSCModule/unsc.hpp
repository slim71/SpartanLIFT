#ifndef __UNSC_HPP__
#define __UNSC_HPP__

#include "LoggerModule/logger.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "types.hpp"
#include "utilities.hpp"
#include <iostream>
#include <math.h>
#include <optional>
#include <rclcpp/rclcpp.hpp>

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

        // External communications
        rclcpp::Time gatherTime() const;
        std::optional<px4_msgs::msg::VehicleGlobalPosition> gatherGlobalPosition() const;
        std::optional<px4_msgs::msg::VehicleOdometry> gatherOdometry() const;
        std::optional<px4_msgs::msg::VehicleCommandAck> gatherAck() const;

    private:
        template<typename... Args> void sendLogInfo(std::string, Args...) const;
        template<typename... Args> void sendLogDebug(std::string, Args...) const;
        template<typename... Args> void sendLogWarning(std::string, Args...) const;
        template<typename... Args> void sendLogError(std::string, Args...) const;

        // void offboardTimerCallback();

        void arm();
        void disarm();
        void takeoff();
        bool waitForAck(uint16_t);

        void signalPublishVehicleCommand(
            uint16_t, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN,
            float = NAN
        ) const;

        bool checkIsRunning();

    private: // Attributes
        Pelican* node_;
        LoggerModule* logger_;

        bool running_ {true};
        mutable std::mutex running_mutex_; // to be used with running_

        rclcpp::TimerBase::SharedPtr timer_;
};

#include "unsc_template.tpp"

#endif
