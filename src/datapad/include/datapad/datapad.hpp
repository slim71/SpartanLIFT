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
        void processContact(rclcpp::Client<comms::srv::FleetInfoExchange>::SharedFuture);
        void inspectAck(const comms::msg::POI);
        void sendFleetInfo(bool, bool, bool);
        void sendPointOfInterest(float, float, float, float);

    private:                                     // Attributes
        LoggerModule logger_;
        static std::weak_ptr<Datapad> instance_; // Weak pointer to the instance of the node

        int leader_ {0};
        bool leader_present_ {false};
        bool fleet_fying_ {false};

        bool running_ {false};
        mutable std::mutex running_mutex_;

        std::unique_ptr<comms::msg::POI> last_msg_;

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

        std::string channel_topic_ {"/datapad/channel"};
        rclcpp::Subscription<comms::msg::POI>::SharedPtr sub_to_channel;
        rclcpp::Publisher<comms::msg::POI>::SharedPtr pub_to_channel;

        std::chrono::seconds setup_timeout_ {constants::SETUP_TIME_SECS};
        rclcpp::TimerBase::SharedPtr setup_timer_;

        rclcpp::Client<comms::srv::FleetInfoExchange>::SharedPtr fleetinfo_client_;
};

// Including templates definitions
#include "templates.tpp"

#endif
