#ifndef _HEARTBEAT_HPP_
#define _HEARTBEAT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include "comms/msg/heartbeat.hpp"
#include "types.hpp"
#include "utilities.hpp"
#include "logger.hpp"

class Pelican;

class HeartbeatModule {
    public:
        explicit HeartbeatModule(Pelican*);
        ~HeartbeatModule();

        void initSetup(LoggerModule*);
        void setupPublisher();
        void setTransmissionTimer();

        void resetSubscription();
        void flushStorage();

        int getNumberOfHbs() const;
        heartbeat getLastHb() const;
        int getMaxHbs() const;

        void sendNow();

    private: // Member functions

        template<typename... Args> void sendLogInfo(std::string, Args...) const;
        template<typename... Args> void sendLogDebug(std::string, Args...) const;
        template<typename... Args> void sendLogWarning(std::string, Args...) const;
        template<typename... Args> void sendLogError(std::string, Args...) const;
        
        void sendHeartbeat() const;
        void stopHeartbeat();
        void storeHeartbeat(const comms::msg::Heartbeat msg);
        void flushHeartbeats();

    private: // Attributes
        Pelican* node_;
        LoggerModule* logger_;

        int max_hbs_;

        std::string heartbeat_topic_ {"/fleet/heartbeat"};
        rclcpp::Subscription<comms::msg::Heartbeat>::SharedPtr sub_to_heartbeat_topic_;
        rclcpp::Publisher<comms::msg::Heartbeat>::SharedPtr pub_to_heartbeat_topic_;

        rmw_qos_profile_t qos_profile_ {rmw_qos_profile_default};
        rclcpp::QoS qos_ {
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile_), qos_profile_)
        };

        std::chrono::milliseconds heartbeat_period_;
        rclcpp::TimerBase::SharedPtr hb_transmission_timer_;

        std::vector<heartbeat> received_hbs_;
        mutable std::mutex hbs_mutex_;
};

#include "heartbeat_templates.tpp"

#endif