#ifndef _CARGO_HPP_
#define _CARGO_HPP_

#include "logger.hpp"
#include "types.hpp"

class Cargo : public rclcpp::Node {
    public:
        // Ctors/Dctors
        explicit Cargo();
        ~Cargo();

        static void signalHandler(int);
        static void setInstance(rclcpp::Node::SharedPtr);
        static std::shared_ptr<Cargo> getInstance();

        bool isRunning() const;

    private: // Member functions
        template<typename... Args> void sendLogInfo(std::string, Args...) const;
        template<typename... Args> void sendLogDebug(std::string, Args...) const;
        template<typename... Args> void sendLogWarning(std::string, Args...) const;
        template<typename... Args> void sendLogError(std::string, Args...) const;

        void storeCargoOdometry(const nav_msgs::msg::Odometry::SharedPtr);

        void setRunning();
        void unsetRunning();

        void
        shareCargoPosition(const std::shared_ptr<comms::srv::CargoPoint::Request>, const std::shared_ptr<comms::srv::CargoPoint::Response>);

    private:                                   // Attributes
        LoggerModule logger_;
        static std::weak_ptr<Cargo> instance_; // Weak pointer to the instance of the node

        geometry_msgs::msg::Point own_odom_ {NAN_point};
        bool running_ {false};

        mutable std::mutex running_mutex_;
        mutable std::mutex odom_mutex_;

        rclcpp::SubscriptionOptions exc_group_opt_ {rclcpp::SubscriptionOptions()};
        rclcpp::CallbackGroup::SharedPtr exc_group_;

        rclcpp::QoS data_qos_ {rclcpp::QoS(
            rclcpp::QoSInitialization(
                rmw_qos_profile_sensor_data.history, constants::QOS_HISTORY_AMOUNT
            ),
            rmw_qos_profile_sensor_data
        )};

        std::string odom_topic_ {"/model/cargo/odometry"};
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_to_odom_;

        rclcpp::Service<comms::srv::CargoPoint>::SharedPtr cargopoint_server_;
};

// Including templates definitions
#include "templates.tpp"

#endif
