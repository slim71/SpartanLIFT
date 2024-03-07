#ifndef _DATAPAD_HPP_
#define _DATAPAD_HPP_

#include "logger.hpp"
#include "types.hpp"

class Datapad : public rclcpp::Node {
    public:
        // Ctors/Dctors
        explicit Datapad();
        ~Datapad();

        static void signalHandler(int);
        static void setInstance(rclcpp::Node::SharedPtr instance);
        static std::shared_ptr<Datapad> getInstance();

        bool isRunning() const;

    private: // Member functions
        template<typename... Args> void sendLogInfo(std::string, Args...) const;
        template<typename... Args> void sendLogDebug(std::string, Args...) const;
        template<typename... Args> void sendLogWarning(std::string, Args...) const;
        template<typename... Args> void sendLogError(std::string, Args...) const;

        void landingPage();
        void contactLeader();
        void unitSortie();
        void backToLZ();
        void payloadExtraction();
        void payloadDropoff();
        void processLeaderResponse(rclcpp::Client<comms::srv::TeleopData>::SharedFuture);
        void sendTeleopData(Flags);

    private:                                     // Attributes
        LoggerModule logger_;
        static std::weak_ptr<Datapad> instance_; // Weak pointer to the instance of the node

        int leader_ {0};
        bool leader_present_ {false};
        bool fleet_fying_ {false};
        bool transport_wip_ {false};

        bool running_ {false};
        mutable std::mutex running_mutex_;

        rclcpp::SubscriptionOptions reentrant_opt_ {rclcpp::SubscriptionOptions()};
        rclcpp::CallbackGroup::SharedPtr reentrant_group_;

        rclcpp::QoS standard_qos_ {rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default), rmw_qos_profile_default
        )};
        rclcpp::QoS data_qos_ {rclcpp::QoS(
            rclcpp::QoSInitialization(
                rmw_qos_profile_sensor_data.history, constants::QOS_HISTORY_AMOUNT
            ),
            rmw_qos_profile_sensor_data
        )};

        std::chrono::seconds setup_timeout_ {constants::SETUP_TIME_SECS};
        rclcpp::TimerBase::SharedPtr setup_timer_;

        rclcpp::Client<comms::srv::TeleopData>::SharedPtr teleopdata_client_;
};

// Including templates definitions
#include "templates.tpp"

#endif
