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
        unsigned int getID() const;
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

        // TODO: differentiate name of funs initiated outside the module
        // from the one initiated BY the module?
        void commenceSetElectionStatus(int);
        void commenceResetElectionTimer();
        void commenceIncreaseCurrentTerm();
        void commenceSetTerm(uint64_t);
        void commenceStopHeartbeatService();
        void commenceStopElectionService();
        void commenceStopTacMapService();
        void commenceStopUNSCService();
        bool commenceSetHome();
        bool commenceTakeoff();
        bool commenceLand();
        bool commenceReturnToLaunchPosition();

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

        void setMass(double);
        void setRole(possible_roles);
        void setTerm(unsigned int);

        int getNetworkSize();

        void parseModel();

        void rollCall();
        void storeAttendance(const comms::msg::Status::SharedPtr);

        void
        rogerWillCo(const std::shared_ptr<comms::srv::FleetInfoExchange::Request>, const std::shared_ptr<comms::srv::FleetInfoExchange::Response>);
        void handleCommand(const comms::msg::Command);
        void broadcastCommand(unsigned int);
        void sendAppendEntryRPC(unsigned int, unsigned int, bool = false, bool = false);
        void sendAck(unsigned int, unsigned int, bool = false);
        bool waitForAcks(unsigned int, bool = false);
        void appendEntry(unsigned int, unsigned int);
        void executeCommand(unsigned int);

    private: // Attributes
        LoggerModule logger_;
        HeartbeatModule hb_core_;
        ElectionModule el_core_;
        TacMapModule tac_core_;
        UNSCModule unsc_core_;

        static std::weak_ptr<Pelican> instance_; // Weak pointer to the instance of the node

        unsigned int id_;
        std::string model_;
        double mass_ {0.0};
        possible_roles role_ {tbd};
        unsigned int current_term_ {0};
        bool ready_ {false};
        int network_size_ {0};
        bool flying_ {false};
        bool mission_in_progress_ {false};

        std::vector<comms::msg::Status> discovery_vector_;
        mutable std::mutex discovery_mutex_; // to be used with discovery_vector_

        rclcpp::SubscriptionOptions reentrant_opt_ {rclcpp::SubscriptionOptions()};
        rclcpp::CallbackGroup::SharedPtr reentrant_group_;

        int qos_value_ = 10;
        rclcpp::QoS qos_ {rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default), rmw_qos_profile_default
        )};
        rclcpp::QoS data_qos_ {rclcpp::QoS(
            rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, 5),
            rmw_qos_profile_sensor_data
        )};

        std::string discovery_topic_ {"/fleet/network"};
        rclcpp::Subscription<comms::msg::Status>::SharedPtr sub_to_discovery_;
        rclcpp::Publisher<comms::msg::Status>::SharedPtr pub_to_discovery_;

        std::string dispatch_topic_ {"/fleet/dispatch"};
        rclcpp::Subscription<comms::msg::Command>::SharedPtr sub_to_dispatch_;
        rclcpp::Publisher<comms::msg::Command>::SharedPtr pub_to_dispatch_;

        std::vector<comms::msg::Command> dispatch_vector_;
        mutable std::mutex dispatch_mutex_;         // to be used with dispatch_vector_

        std::chrono::seconds rollcall_timeout_ {1}; // TODO: constant/parameter
        rclcpp::TimerBase::SharedPtr rollcall_timer_;

        rclcpp::Service<comms::srv::FleetInfoExchange>::SharedPtr fleetinfo_server_;

        std::chrono::seconds ack_timeout_ {1}; // TODO: constant/parameter

        std::vector<std::tuple<unsigned int, unsigned int>> entries_rpcs_;
        mutable std::mutex entries_mutex_;
};

// Including templates definitions
#include "pelican_templates.tpp"

#endif
