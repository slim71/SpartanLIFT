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
        void recordLeaderPosition(const nav_msgs::msg::Odometry::SharedPtr);

        void
        shareCargoPosition(const std::shared_ptr<comms::srv::CargoPoint::Request>, const std::shared_ptr<comms::srv::CargoPoint::Response>);
        void
        notifyAttachment(const std::shared_ptr<comms::srv::CargoLinkage::Request>, const std::shared_ptr<comms::srv::CargoLinkage::Response>);
        void parseSetEntityPoseResponse(
            rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedFuture
        );

        void followReference();
        void startFollowing();
        void stopFollowing();

        void setRunning();
        void unsetRunning();

    private:                                   // Attributes
        LoggerModule logger_;
        static std::weak_ptr<Cargo> instance_; // Weak pointer to the instance of the node

        std::string model_name_;
        std::string world_name_;
        bool running_ {false};
        geometry_msgs::msg::Point own_odom_ {NAN_point};
        boost::circular_buffer<nav_msgs::msg::Odometry> reference_buffer_ {
            constants::REFERENCE_BUFFER_SIZE};

        mutable std::mutex running_mutex_;
        mutable std::mutex odom_mutex_;
        mutable std::mutex reference_mutex_;

        rclcpp::SubscriptionOptions own_odom_exc_group_opt_ {rclcpp::SubscriptionOptions()};
        rclcpp::CallbackGroup::SharedPtr own_odom_exc_group_;

        rclcpp::SubscriptionOptions leader_odom_exc_group_opt_ {rclcpp::SubscriptionOptions()};
        rclcpp::CallbackGroup::SharedPtr leader_odom_exc_group_;

        rclcpp::SubscriptionOptions pose_exc_group_opt_ {rclcpp::SubscriptionOptions()};
        rclcpp::CallbackGroup::SharedPtr pose_exc_group_;

        rclcpp::QoS data_qos_ {rclcpp::QoS(
            rclcpp::QoSInitialization(
                rmw_qos_profile_sensor_data.history, constants::QOS_HISTORY_AMOUNT
            ),
            rmw_qos_profile_sensor_data
        )};

        std::string odom_topic_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_to_odom_;

        std::string set_pose_service_;
        rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr set_pose_client_;

        std::string leader_odom_topic_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_to_leader_odometry_topic_;

        rclcpp::Service<comms::srv::CargoPoint>::SharedPtr cargopoint_server_;
        rclcpp::Service<comms::srv::CargoLinkage>::SharedPtr attachment_server_;

        rclcpp::TimerBase::SharedPtr following_timer_;
        std::chrono::milliseconds following_timeout_ {constants::FOLLOW_TIME_MILLISECS};
};

// Including templates definitions
#include "templates.tpp"

#endif
