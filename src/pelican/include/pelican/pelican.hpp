#ifndef _PELICAN_HPP_
#define _PELICAN_HPP_

#include "px4_msgs/msg/vehicle_local_position.hpp"
#include <chrono>
#include <iostream>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <string>
#include "types.hpp"
#include "logger.hpp"
#include "HeartbeatModule/heartbeat.hpp"
#include "ElectionModule/election.hpp"

using std::literals::string_literals::operator""s;

class Pelican : public rclcpp::Node {
    public:
        explicit Pelican();
        ~Pelican();

        int getID() const;
        std::string getModel() const;
        double getMass() const;
        possible_roles getRole() const;
        int getCurrentTerm() const;
        rclcpp::SubscriptionOptions getReentrantOptions() const;
        rclcpp::CallbackGroup::SharedPtr getReentrantGroup() const;

        static void signalHandler(int signum);
        static void setInstance(rclcpp::Node::SharedPtr instance);
        static std::shared_ptr<Pelican> getInstance();

        bool isLeader() const;
        bool isFollower() const;
        bool isCandidate() const;

        void commenceFollowerOperations();
        void commenceLeaderOperations();
        void commenceCandidateOperations();

        void increaseCurrentTerm();

        heartbeat requestLastHb();
        int requestNumberOfHbs();
        void requestSetElectionStatus(int);
        void requestResetElectionTimer();

    private: // Member functions
        LoggerModule logger_;
        HeartbeatModule hb_core_;
        ElectionModule el_core_;

        template<typename... Args> void sendLogInfo(std::string, Args...) const;
        template<typename... Args> void sendLogDebug(std::string, Args...) const;
        template<typename... Args> void sendLogWarning(std::string, Args...) const;
        template<typename... Args> void sendLogError(std::string, Args...) const;

        void becomeLeader();
        void becomeFollower();
        void becomeCandidate();

        void setRole(possible_roles r);

        void parseModel();

        void printData(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) const;

        void setMass(double m);

    private: // Attributes
        int id_;
        std::string model_;
        double mass_ {0.0};

        possible_roles role_ {tbd};

        int current_term_ {0};
        static std::weak_ptr<Pelican> instance_; // Weak pointer to the instance of the node

        // The subscription sets a QoS profile based on rmw_qos_profile_sensor_data.
        // This is needed because the default ROS 2 QoS profile for subscribers is
        // incompatible with the PX4 profile for publishers.
        rclcpp::QoS px4_qos_ {rclcpp::QoS(
            rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, 5),
            rmw_qos_profile_sensor_data
        )};
        rmw_qos_profile_t qos_profile_ {rmw_qos_profile_default};
        rclcpp::QoS standard_qos_ {
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile_), qos_profile_)
        };

        rclcpp::SubscriptionOptions reentrant_opt_ {rclcpp::SubscriptionOptions()};
        rclcpp::CallbackGroup::SharedPtr reentrant_group_;

        std::string local_pos_topic_; // i.e. "/fmu/out/vehicle_local_position";
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
            sub_to_local_pos_topic_;

};

// Including templates definitions
#include "pelican_templates.tpp"

#endif
