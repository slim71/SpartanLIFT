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
        static void setInstance(rclcpp::Node::SharedPtr);
        static std::shared_ptr<Datapad> getInstance();

        bool isRunning() const;
        TriState isCargoHandled() const;

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
        void askForCargoPoint();
        void storeCargoPoint(rclcpp::Client<comms::srv::CargoPoint>::SharedFuture);
        void processFleetLeaderCommandAck(const rclcpp_action::ClientGoalHandle<
                                          comms::action::TeleopData>::WrappedResult&);
        void teleopTaskClient(Flags);
        void analyzeTeleopDataResponse(const rclcpp_action::ClientGoalHandle<
                                       comms::action::TeleopData>::SharedPtr&);
        void
        parseTeleopDataFeedback(rclcpp_action::ClientGoalHandle<comms::action::TeleopData>::SharedPtr, const std::shared_ptr<const comms::action::TeleopData::Feedback>);

        void setAndNotifyCargoHandled();
        void unsetAndNotifyCargoHandled();

    private:                                     // Attributes
        LoggerModule logger_;
        static std::weak_ptr<Datapad> instance_; // Weak pointer to the instance of the node

        int leader_ {0};
        bool leader_present_ {false};
        bool fleet_fying_ {false};
        bool transport_wip_ {false};
        bool running_ {false};
        geometry_msgs::msg::Point cargo_odom_ {NAN_point};
        std::condition_variable cargo_cv_;
        TriState cargo_handled_ {TriState::Floating};

        mutable std::mutex running_mutex_;        // to be used with running_
        mutable std::mutex cargo_mutex_;          // to be used with cargo_odom_
        mutable std::mutex cargo_tristate_mutex_; // to be used with cargo_handled_

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

        rclcpp_action::Client<comms::action::TeleopData>::SharedPtr teleopdata_client_;
        rclcpp::Client<comms::srv::CargoPoint>::SharedPtr cargopoint_client_;
};

// Including templates definitions
#include "templates.tpp"

#endif
