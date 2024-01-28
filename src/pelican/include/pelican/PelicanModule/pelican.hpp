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

        // Core functionalities
        static void signalHandler(int signum);
        static void setInstance(rclcpp::Node::SharedPtr instance);
        static std::shared_ptr<Pelican> getInstance();

        // Actions initiated by the module
        bool initiateSetHome();
        bool initiateArm();
        bool initiateTakeoff();
        bool initiateLand();
        bool initiateReturnToLaunchPosition();

        // Getters
        unsigned int getID() const;
        std::string getModel() const;
        double getMass() const;
        possible_roles getRole() const;
        unsigned int getCurrentTerm() const;
        rclcpp::SubscriptionOptions getReentrantOptions() const;
        rclcpp::CallbackGroup::SharedPtr getReentrantGroup() const;
        rclcpp::Time getTime() const;
        int getNetworkSize() const;

        // Status check
        bool isLeader() const;
        bool isFollower() const;
        bool isCandidate() const;
        bool isReady() const;
        bool isFlying() const;

        // Handle data exchange among modules
        heartbeat requestLastHb();
        int requestNumberOfHbs();
        std::optional<px4_msgs::msg::VehicleGlobalPosition> requestGlobalPosition();
        std::optional<px4_msgs::msg::VehicleOdometry> requestOdometry();
        std::optional<px4_msgs::msg::VehicleCommandAck> requestAck();
        std::optional<px4_msgs::msg::VehicleStatus> requestStatus();

        // Actions initiated from outside the module
        void commenceFollowerOperations();  // Election and Heartbeat modules
        void commenceLeaderOperations();    // Election module
        void commenceCandidateOperations(); // Election module
        void commencePublishVehicleCommand(
            uint16_t, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN,
            float = NAN
        );                                                                  // UNSC module
        void commencePublishOffboardControlMode();                          // UNSC module
        void commencePublishTrajectorySetpoint(float, float, float, float); // UNSC module
        void commenceSetElectionStatus(int);                                // Heartbeat module
        void commenceResetElectionTimer();                                  // Heartbeat module
        void commenceIncreaseCurrentTerm();                                 // Election module
        void commenceSetTerm(uint64_t); // Election and Heartbeat modules

        // To stop modules; some are not actively used but kept for possible future use
        void commenceStopHeartbeatService();
        void commenceStopElectionService();
        void commenceStopTacMapService();
        void commenceStopUNSCService();

    private: // Member functions
        template<typename... Args> void sendLogInfo(std::string, Args...) const;
        template<typename... Args> void sendLogDebug(std::string, Args...) const;
        template<typename... Args> void sendLogWarning(std::string, Args...) const;
        template<typename... Args> void sendLogError(std::string, Args...) const;

        void parseModel();
        void rollCall();
        void storeAttendance(const comms::msg::Status::SharedPtr);

        void becomeLeader();
        void becomeFollower();
        void becomeCandidate();

        void setID(unsigned int);
        void setMass(double);
        void setRole(possible_roles);
        void setTerm(unsigned int);
        void setFlyingStatus();
        void unsetFlyingStatus();

        void
        rogerWillCo(const std::shared_ptr<comms::srv::FleetInfoExchange::Request>, const std::shared_ptr<comms::srv::FleetInfoExchange::Response>);
        bool checkCommandMsgValidity(const comms::msg::Command);
        void handleCommandDispatch(unsigned int);
        void handleCommandReception(const comms::msg::Command);
        bool broadcastCommand(unsigned int);
        void sendAppendEntryRPC(unsigned int, unsigned int, bool = false, bool = false);
        void sendAck(unsigned int, unsigned int, bool = false);
        bool waitForAcks(unsigned int, bool = false);
        void appendEntry(unsigned int, unsigned int);
        bool executeCommand(unsigned int);

    private: // Attributes
        LoggerModule logger_;
        HeartbeatModule hb_core_;
        ElectionModule el_core_;
        TacMapModule tac_core_;
        UNSCModule unsc_core_;

        // Weak pointer to the instance of the node
        static std::weak_ptr<Pelican> instance_;

        unsigned int id_;
        unsigned int current_term_ {0};
        double mass_ {0.0};
        bool ready_ {false};
        bool flying_ {false};
        bool mission_in_progress_ {false};
        std::string model_;
        possible_roles role_ {tbd};

        mutable std::mutex id_mutex_;     // Used to access id_
        mutable std::mutex term_mutex_;   // Used to access current_term_
        mutable std::mutex flying_mutex_; // Used to access flying_

        rclcpp::SubscriptionOptions reentrant_opt_ {rclcpp::SubscriptionOptions()};
        rclcpp::CallbackGroup::SharedPtr reentrant_group_;

        rclcpp::QoS qos_ {rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default), rmw_qos_profile_default
        )};
        rclcpp::QoS data_qos_ {rclcpp::QoS(
            rclcpp::QoSInitialization(
                rmw_qos_profile_sensor_data.history, constants::QOS_HISTORY_AMOUNT
            ),
            rmw_qos_profile_sensor_data
        )};

        rclcpp::Service<comms::srv::FleetInfoExchange>::SharedPtr fleetinfo_server_;

        std::string discovery_topic_ {"/fleet/network"};
        rclcpp::Subscription<comms::msg::Status>::SharedPtr sub_to_discovery_;
        rclcpp::Publisher<comms::msg::Status>::SharedPtr pub_to_discovery_;

        std::string dispatch_topic_ {"/fleet/dispatch"};
        rclcpp::Subscription<comms::msg::Command>::SharedPtr sub_to_dispatch_;
        rclcpp::Publisher<comms::msg::Command>::SharedPtr pub_to_dispatch_;

        std::vector<comms::msg::Status> discovery_vector_;
        mutable std::mutex discovery_mutex_; // To be used with discovery_vector_

        std::vector<comms::msg::Command> dispatch_vector_;
        mutable std::mutex dispatch_mutex_; // To be used with dispatch_vector_

        std::vector<std::tuple<unsigned int, unsigned int>> rpcs_vector_;
        mutable std::mutex rpcs_mutex_;

        std::chrono::seconds rollcall_timeout_ {constants::ROLLCALL_TIME_SECS};
        rclcpp::TimerBase::SharedPtr rollcall_timer_;

        std::chrono::seconds ack_timeout_ {constants::ACK_TIME_SECS};
};

// Including templates definitions
#include "pelican_templates.tpp"

#endif
