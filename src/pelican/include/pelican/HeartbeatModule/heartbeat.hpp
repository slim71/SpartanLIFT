#ifndef _HEARTBEAT_HPP_
#define _HEARTBEAT_HPP_

#include "LoggerModule/logger.hpp"
#include "types.hpp"
#include "utilities.hpp"

class Pelican;

class HeartbeatModule {
    public:
        // Ctors/Dctors
        explicit HeartbeatModule();
        explicit HeartbeatModule(Pelican*);
        ~HeartbeatModule();

        // Setup methods
        void initSetup(LoggerModule*);
        void setupPublisher();
        void setupSubscription();
        void setupTransmissionTimer();

        // Actions initiated from outside the module
        void resetPublisher();

        // Both from outside and inside the module
        void flushHeartbeats();

        // Special functionalities
        void sendNow();
        void stopService();

        int getNumberOfHbs() const;
        heartbeat getLastHb() const;
        int getMaxHbs() const;

    private: // Member functions
        template<typename... Args> void sendLogInfo(std::string, Args...) const;
        template<typename... Args> void sendLogDebug(std::string, Args...) const;
        template<typename... Args> void sendLogWarning(std::string, Args...) const;
        template<typename... Args> void sendLogError(std::string, Args...) const;

        // Core functionalities
        void sendHeartbeat() const;
        void storeHeartbeat(const comms::msg::Heartbeat);
        bool checkHeartbeatValidity(const comms::msg::Heartbeat);

        // External communications
        unsigned int gatherAgentID() const;
        possible_roles gatherAgentRole() const;
        unsigned int gatherCurrentTerm() const;
        rclcpp::Time gatherTime() const;
        rclcpp::CallbackGroup::SharedPtr gatherReentrantGroup() const;
        rclcpp::SubscriptionOptions gatherReentrantOptions() const;
        void signalTransitionToFollower() const;
        void signalSetElectionStatus(int64_t);
        void signalResetElectionTimer();
        void signalSetTerm(uint64_t term) const;

    private: // Attributes
        Pelican* node_;
        LoggerModule* logger_;

        int max_hbs_ {constants::MAX_HB_NUM};

        std::string heartbeat_topic_ {"/fleet/heartbeat"};
        rclcpp::Subscription<comms::msg::Heartbeat>::SharedPtr sub_to_heartbeat_topic_;
        rclcpp::Publisher<comms::msg::Heartbeat>::SharedPtr pub_to_heartbeat_topic_;

        rclcpp::QoS qos_ {rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default), rmw_qos_profile_default
        )};

        // Not completely random: it has to be lower than the election_timeout_
        std::chrono::milliseconds heartbeat_period_ {constants::HB_REPETITION_PERIOD_MILLIS};
        rclcpp::TimerBase::SharedPtr hb_transmission_timer_;

        std::vector<heartbeat> received_hbs_;
        mutable std::mutex hbs_mutex_;
};

#include "heartbeat_templates.tpp"

#endif
