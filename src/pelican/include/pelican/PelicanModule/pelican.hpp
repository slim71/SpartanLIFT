#ifndef _PELICAN_HPP_
#define _PELICAN_HPP_

#include "ElectionModule/election.hpp"
#include "HeartbeatModule/heartbeat.hpp"
#include "LoggerModule/logger.hpp"
#include "TacMapModule/tacmap.hpp"
#include "UNSCModule/unsc.hpp"
#include "types.hpp"

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
        unsigned int getCurrentTerm() const;
        rclcpp::SubscriptionOptions getReentrantOptions() const;
        rclcpp::CallbackGroup::SharedPtr getReentrantGroup() const;
        rclcpp::Time getTime() const;

        // Core functionalities
        static void signalHandler(int signum);
        static void setInstance(rclcpp::Node::SharedPtr instance);
        static std::shared_ptr<Pelican> getInstance();

        // Actions initiated from outside the module
        void commenceFollowerOperations();
        void commenceLeaderOperations();
        void commenceCandidateOperations();
        void commencePublishVehicleCommand(
            uint16_t, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN,
            float = NAN
        );
        void commencePublishOffboardControlMode();
        void commencePublishTrajectorySetpoint(float, float, float, float);

        // Handle data exchange among modules
        heartbeat requestLastHb();
        int requestNumberOfHbs();
        std::optional<px4_msgs::msg::VehicleGlobalPosition> requestGlobalPosition();
        std::optional<px4_msgs::msg::VehicleOdometry> requestOdometry();
        std::optional<px4_msgs::msg::VehicleCommandAck> requestAck();
        std::optional<px4_msgs::msg::VehicleStatus> requestStatus();
        int requestNetworkSize();

        void commenceSetElectionStatus(int);
        void commenceResetElectionTimer();
        void commenceIncreaseCurrentTerm();
        void commenceStopHeartbeatService();
        void commenceStopElectionService();
        void commenceStopTacMapService();
        void commenceStopUNSCService();

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

        int getNetworkSize();

        void parseModel();

        void rollCall();
        void storeAttendance(const comms::msg::Status::SharedPtr);

    private: // Attributes
        LoggerModule logger_;
        HeartbeatModule hb_core_;
        ElectionModule el_core_;
        TacMapModule tac_core_;
        UNSCModule unsc_core_;

        static std::weak_ptr<Pelican> instance_; // Weak pointer to the instance of the node

        int id_;
        std::string model_;
        double mass_ {0.0};
        possible_roles role_ {tbd};
        unsigned int current_term_ {0};
        bool ready_ {false};
        int network_size_ {0};

        std::vector<comms::msg::Status> discovery_vector_;
        mutable std::mutex discovery_mutex_; // to be used with discovery_vector_

        rclcpp::SubscriptionOptions reentrant_opt_ {rclcpp::SubscriptionOptions()};
        rclcpp::CallbackGroup::SharedPtr reentrant_group_;

        int qos_value_ = 10;
        rmw_qos_profile_t qos_profile_ {rmw_qos_profile_default};
        rclcpp::QoS qos_ {
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile_), qos_profile_)};
        rclcpp::QoS data_qos_ {rclcpp::QoS(
            rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, 5),
            rmw_qos_profile_sensor_data
        )};

        std::string discovery_topic_ {"/fleet/network"};
        rclcpp::Subscription<comms::msg::Status>::SharedPtr sub_to_discovery;
        rclcpp::Publisher<comms::msg::Status>::SharedPtr pub_to_discovery;

        std::chrono::seconds rollcall_timeout_ {1};
        rclcpp::TimerBase::SharedPtr rollcall_timer_;
};

// Including templates definitions
#include "pelican_templates.tpp"

#endif
