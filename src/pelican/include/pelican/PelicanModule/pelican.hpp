#ifndef _PELICAN_HPP_
#define _PELICAN_HPP_

#include "ElectionModule/election.hpp"
#include "HeartbeatModule/heartbeat.hpp"
#include "LoggerModule/logger.hpp"
#include "TacMapModule/tacmap.hpp"
#include "UNSCModule/unsc.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "types.hpp"
#include <chrono>
#include <iostream>
#include <optional> // CHECK: move to types.hpp?
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <string>

class Pelican : public rclcpp::Node {
    public:
        // Ctors/Dctors
        explicit Pelican();
        ~Pelican();

        // Getters
        int getID() const;
        std::string getModel() const;
        double getMass() const;
        possible_roles getRole() const;
        int getCurrentTerm() const;
        rclcpp::SubscriptionOptions getReentrantOptions() const;
        rclcpp::CallbackGroup::SharedPtr getReentrantGroup() const;

        // Core functionalities
        static void signalHandler(int signum);
        static void setInstance(rclcpp::Node::SharedPtr instance);
        static std::shared_ptr<Pelican> getInstance();

        // Actions initiated from outside the module
        void commenceFollowerOperations();
        void commenceLeaderOperations();
        void commenceCandidateOperations();
        void commenceIsTerminated();
        void commencePublishVehicleCommand(
            uint16_t, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN,
            float = NAN
        );

        // Handle data exchange among modules
        heartbeat requestLastHb();
        int requestNumberOfHbs();
        std::optional<px4_msgs::msg::VehicleGlobalPosition> requestGlobalPosition();
        std::optional<px4_msgs::msg::VehicleOdometry> requestOdometry();
        std::optional<px4_msgs::msg::VehicleCommandAck> requestAck();

        void commenceSetElectionStatus(int);
        void commenceResetElectionTimer();
        void commenceIncreaseCurrentTerm();
        void commenceStopHeartbeat();
        void commenceStopBallotThread();

        bool isLeader() const;
        bool isFollower() const;
        bool isCandidate() const;
        bool isReady() const;

    private: // Member functions
        template<typename... Args> void sendLogInfo(std::string, Args...) const;
        template<typename... Args> void sendLogDebug(std::string, Args...) const;
        template<typename... Args> void sendLogWarning(std::string, Args...) const;
        template<typename... Args> void sendLogError(std::string, Args...) const;

        void becomeLeader();
        void becomeFollower();
        void becomeCandidate();

        void setMass(double m);
        void setRole(possible_roles r);

        void parseModel();

    private: // Attributes
        LoggerModule logger_;
        HeartbeatModule hb_core_;
        ElectionModule el_core_;
        TacMapModule tac_core_;
        UNSCModule unsc_core_;

        int id_;
        std::string model_;
        double mass_ {0.0};
        possible_roles role_ {tbd};
        int current_term_ {0};
        static std::weak_ptr<Pelican> instance_; // Weak pointer to the instance of the node
        bool ready_ {false};

        rclcpp::SubscriptionOptions reentrant_opt_ {rclcpp::SubscriptionOptions()};
        rclcpp::CallbackGroup::SharedPtr reentrant_group_;
};

// Including templates definitions
#include "pelican_templates.tpp"

#endif
