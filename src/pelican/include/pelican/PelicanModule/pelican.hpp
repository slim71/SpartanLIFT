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

        // Getters
        unsigned int getID() const;
        std::string getModel() const;
        double getMass() const;
        possible_roles getRole() const;
        double getROI() const;
        unsigned int getCurrentTerm() const;
        rclcpp::Time getTime() const;
        unsigned int getNetworkSize() const;
        geometry_msgs::msg::Point getCopterPosition(unsigned int) const;
        std::vector<unsigned int> getCoptersIDs() const;
        geometry_msgs::msg::Point getDesiredPosition() const;
        geometry_msgs::msg::Point getNeighborDesiredPosition() const;
        geometry_msgs::msg::Point getDropoffPosition() const;

        // For callback groups
        rclcpp::SubscriptionOptions getReentrantOptions() const;
        rclcpp::CallbackGroup::SharedPtr getReentrantGroup() const;
        // Each callback in these groups should not execute in parallel with itself,
        // but can be parallel to one another
        rclcpp::CallbackGroup::SharedPtr getTimerExclusiveGroup() const;
        rclcpp::CallbackGroup::SharedPtr getOffboardExclusiveGroup() const;
        rclcpp::CallbackGroup::SharedPtr getRendezvousExclusiveGroup() const;
        rclcpp::CallbackGroup::SharedPtr getFormationExclusiveGroup() const;
        rclcpp::CallbackGroup::SharedPtr getFormationTimerGroup() const;

        // Flags checks
        bool isLeader() const;
        bool isFollower() const;
        bool isCandidate() const;
        bool isReady() const;
        bool isFlying() const;
        bool isCarrying() const;
        bool isLastCmdExecuted() const;
        bool isFormationAchieved() const;

        // Handle data exchange among modules
        unsigned int requestLeaderID() const;                                   // Election module
        heartbeat requestLastHb() const;                                        // Heartbeat module
        int requestNumberOfHbs() const;                                         // Heartbeat module
        std::optional<nav_msgs::msg::Odometry> requestENUOdometry() const;      // TacMap module
        std::optional<px4_msgs::msg::VehicleStatus> requestStatus() const;      // TacMap module
        double requestActualTargetHeight() const;                               // UNSC module
        std::optional<geometry_msgs::msg::Point> requestTargetPosition() const; // UNSC module

        // Actions initiated by the module
        bool initiateSetHome();                                                       // UNSC module
        bool initiateArm();                                                           // UNSC module
        bool initiateTakeoff();                                                       // UNSC module
        bool initiateLand();                                                          // UNSC module
        bool initiateReturnToLaunchPosition();                                        // UNSC module
        void initiateConsensus();                                                     // UNSC module
        void initiateSetPositionSetpoint(geometry_msgs::msg::Point);                  // UNSC module
        std::optional<geometry_msgs::msg::Point> initiateGetPositionSetpoint() const; // UNSC module
        void initiateSetTargetPosition(geometry_msgs::msg::Point);                    // UNSC module
        void initiateUnblockFormation();                                              // UNSC module
        void initiatePreFormationActions();                                           // UNSC module

        // Actions initiated from outside the module
        void commenceSetElectionStatus(int); // Heartbeat module
        void commenceResetElectionTimer();   // Heartbeat module
        void commenceSetTerm(uint64_t);      // Election and Heartbeat modules
        void commenceFollowerOperations();   // Election and Heartbeat modules
        void commenceIncreaseCurrentTerm();  // Election module
        void commenceLeaderOperations();     // Election module
        void commenceCandidateOperations();  // Election module
        void commencePublishVehicleCommand(
            uint16_t, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN,
            float = NAN
        );                                         // UNSC module
        void commencePublishOffboardControlMode(); // UNSC module
        void commencePublishTrajectorySetpoint(
            geometry_msgs::msg::Point, geometry_msgs::msg::Point
        );                                          // UNSC module
        void commenceSetActualTargetHeight(double); // UNSC module
        void commenceNotifyAgentInFormation();      // UNSC module
        void commenceSyncTrigger();                 // UNSC module
        void commenceSendDesiredFormationPositions(std::unordered_map<
                                                   unsigned int,
                                                   geometry_msgs::msg::Point>); // UNSC module
        void commenceAskDesPosToNeighbor(unsigned int);                         // UNSC module
        void commenceCargoAttachment();                                         // UNSC module
        void commenceUnsetCarryingStatus();
        bool commenceCheckOffboardEngagement();                // TacMap and UNSC modules
        bool commenceWaitForCommanderAck(uint16_t);            // TacMap module
        void commenceHeightCompensation(double);               // TacMap module
        void commenceSharePosition(geometry_msgs::msg::Point); // TacMap module

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

        // Other functionalities
        void parseModel();
        void storeAttendance(comms::msg::NetworkVertex::SharedPtr);
        void recordCopterPosition(comms::msg::NetworkVertex::SharedPtr);
        void becomeLeader();
        void becomeFollower();
        void becomeCandidate();

        // Command-related
        bool checkCommandMsgValidity(const comms::msg::Command);
        void handleCommandDispatch(uint16_t);
        bool broadcastCommand(uint16_t);
        void sendRPCAck(unsigned int, uint16_t, bool = false);
        bool waitForRPCsAcks(uint16_t, bool = false);
        void appendEntry(uint16_t, unsigned int);
        bool executeRPCCommand(uint16_t);

        // Topic-related
        // "/fleet/network"
        void sharePosition(geometry_msgs::msg::Point);                    // publisher
        void storeCopterInfo(const comms::msg::NetworkVertex::SharedPtr); // subscriber
        // "/fleet/dispatch"
        void handleCommandReception(const comms::msg::Command);                      // subscriber
        void sendAppendEntryRPC(unsigned int, uint16_t, bool = false, bool = false); // publisher
        // "/fleet/formation"
        void storeDesiredPosition(const comms::msg::FormationDesired);                // subscriber
        void sendDesiredFormationPositions(std::unordered_map<
                                           unsigned int, geometry_msgs::msg::Point>); // publisher
        // "/fleet/synchronization"
        void handleSyncSignal(std_msgs::msg::Empty); // subscriber
        void syncTrigger();                          // publisher

        // Action server-related
        // TeleopData action
        rclcpp_action::GoalResponse
        handleTeleopDataGoal(const rclcpp_action::GoalUUID&, std::shared_ptr<const comms::action::TeleopData::Goal>);
        rclcpp_action::CancelResponse
        handleTeleopDataCancellation(const std::shared_ptr<
                                     rclcpp_action::ServerGoalHandle<comms::action::TeleopData>>);
        void handleAcceptedTeleopData(const std::shared_ptr<
                                      rclcpp_action::ServerGoalHandle<comms::action::TeleopData>>);
        void rogerWillCo(const std::shared_ptr<
                         rclcpp_action::ServerGoalHandle<comms::action::TeleopData>>);

        // Service-related
        // FleetInfo
        void
        targetNotification(const std::shared_ptr<comms::srv::FleetInfo::Request>, const std::shared_ptr<comms::srv::FleetInfo::Response>);
        void rendezvousFleet();
        void processLeaderResponse(rclcpp::Client<comms::srv::FleetInfo>::SharedFuture);
        void
        shareDesiredPosition(const std::shared_ptr<comms::srv::FleetInfo::Request>, std::shared_ptr<comms::srv::FleetInfo::Response>);
        void askDesPosToNeighbor(unsigned int);
        void storeNeighborDesPos(rclcpp::Client<comms::srv::FleetInfo>::SharedFuture);
        // CargoLinkage
        void checkCargoAttachment(rclcpp::Client<comms::srv::CargoLinkage>::SharedFuture);
        void cargoAttachment();
        // FormationReached
        void
        recordAgentInFormation(const std::shared_ptr<comms::srv::FormationReached::Request>, std::shared_ptr<comms::srv::FormationReached::Response>);
        void notifyAgentInFormation();

        // Setters
        void setID(unsigned int);
        void setMass(double);
        void setRole(possible_roles);
        void setTerm(unsigned int);
        void setNeighborPosition(geometry_msgs::msg::Point);
        void setDropoffPosition(geometry_msgs::msg::Point);
        // Flags
        void setFlyingStatus();
        void unsetFlyingStatus();
        void setCarryingStatus();
        void unsetCarryingStatus();
        void setLastCmdStatus();
        void unsetLastCmdStatus();
        void setAndNotifyRendezvousHandled();
        void unsetAndNotifyRendezvousHandled();
        void setAndNotifyFormationHandled();
        void unsetAndNotifyFormationHandled();
        void setFormationAchieved();
        void unsetFormationAchieved();

    private: // Attributes
        LoggerModule logger_;
        HeartbeatModule hb_core_;
        ElectionModule el_core_;
        TacMapModule tac_core_;
        UNSCModule unsc_core_;

        // Weak pointer to the instance of the node
        static std::weak_ptr<Pelican> instance_;

        bool ready_ {false};
        bool flying_ {false};
        bool carrying_ {false};
        bool mission_in_progress_ {false};
        bool last_cmd_result_ {false};
        bool formation_achieved_ {false};
        unsigned int id_;
        unsigned int current_term_ {0};
        double mass_ {0.0};
        double roi_;
        std::string model_;
        // Map with pairs: agent ID - agent position
        std::unordered_map<unsigned int, geometry_msgs::msg::Point> copters_positions_;
        std::vector<std::tuple<unsigned int, unsigned int>> rpcs_vector_;
        std::vector<comms::msg::NetworkVertex> discovery_vector_;
        std::vector<comms::msg::Command> dispatch_vector_;
        std::vector<unsigned int> agents_in_formation_;
        geometry_msgs::msg::Point des_formation_pos_ = NAN_point;
        geometry_msgs::msg::Point neigh_des_pos_ = NAN_point;
        geometry_msgs::msg::Point dropoff_position_ = NAN_point;
        std::shared_ptr<rclcpp_action::ServerGoalHandle<comms::action::TeleopData>>
            last_goal_handle_;
        std::condition_variable rendezvous_handled_cv_;
        std::condition_variable formation_handled_cv_;
        possible_roles role_ {tbd};
        TriState rendezvous_handled_ {TriState::Floating};
        TriState formation_handled_ {TriState::Floating};

        mutable std::mutex id_mutex_;              // Used to access id_
        mutable std::mutex term_mutex_;            // Used to access current_term_
        mutable std::mutex flying_mutex_;          // Used to access flying_
        mutable std::mutex carrying_mutex_;        // Used to access carrying_
        mutable std::mutex positions_mutex_;       // to be used with copters_positions_
        mutable std::mutex discovery_mutex_;       // To be used with discovery_vector_
        mutable std::mutex dispatch_mutex_;        // To be used with dispatch_vector_
        mutable std::mutex rpcs_mutex_;            // To be used with rpcs_vector_
        mutable std::mutex last_cmd_result_mutex_; // To be used with last_cmd_result_
        mutable std::mutex last_goal_mutex_;       // To be used with last_goal_handle_
        mutable std::mutex rendez_tristate_mutex_; // to be used with rendezvous_handled_
        mutable std::mutex form_tristate_mutex_;   // to be used with rendezvous_handled_
        mutable std::mutex formation_mutex_;       // to be used with des_formation_pos_
        mutable std::mutex neigh_mutex_;           // to be used with neigh_des_pos_
        mutable std::mutex dropoff_mutex_;         // to be used with neigh_des_pos_
        mutable std::mutex form_reached_mutex_;    // to be used with agents_in_formation_
        mutable std::mutex form_achieved_mutex_;   // to be used with agents_in_formation_

        rclcpp::SubscriptionOptions reentrant_opt_ {rclcpp::SubscriptionOptions()};
        rclcpp::CallbackGroup::SharedPtr reentrant_group_;
        rclcpp::SubscriptionOptions timer_exclusive_opt_ {rclcpp::SubscriptionOptions()};
        rclcpp::CallbackGroup::SharedPtr timer_exclusive_group_;
        rclcpp::SubscriptionOptions offboard_exclusive_opt_ {rclcpp::SubscriptionOptions()};
        rclcpp::CallbackGroup::SharedPtr offboard_exclusive_group_;
        rclcpp::SubscriptionOptions rendezvous_exclusive_opt_ {rclcpp::SubscriptionOptions()};
        rclcpp::CallbackGroup::SharedPtr rendezvous_exclusive_group_;
        rclcpp::SubscriptionOptions formation_exclusive_opt_ {rclcpp::SubscriptionOptions()};
        rclcpp::CallbackGroup::SharedPtr formation_exclusive_group_;
        rclcpp::SubscriptionOptions formation_timer_opt_ {rclcpp::SubscriptionOptions()};
        rclcpp::CallbackGroup::SharedPtr formation_timer_group_;

        rclcpp::QoS qos_ {rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default), rmw_qos_profile_default
        )};
        rclcpp::QoS data_qos_ {rclcpp::QoS(
            rclcpp::QoSInitialization(
                rmw_qos_profile_sensor_data.history, constants::QOS_HISTORY_AMOUNT
            ),
            rmw_qos_profile_sensor_data
        )};

        rclcpp_action::Server<comms::action::TeleopData>::SharedPtr teleopdata_server_;

        rclcpp::Service<comms::srv::FleetInfo>::SharedPtr fleetinfo_server_;
        rclcpp::Client<comms::srv::FleetInfo>::SharedPtr fleetinfo_client_;

        rclcpp::Client<comms::srv::CargoLinkage>::SharedPtr cargo_attachment_client_;

        rclcpp::Service<comms::srv::FleetInfo>::SharedPtr des_pos_server_;
        rclcpp::Client<comms::srv::FleetInfo>::SharedPtr des_pos_client_;

        rclcpp::Service<comms::srv::FormationReached>::SharedPtr form_reached_server_;
        rclcpp::Client<comms::srv::FormationReached>::SharedPtr form_reached_client_;

        std::string dispatch_topic_ {"/fleet/dispatch"};
        rclcpp::Subscription<comms::msg::Command>::SharedPtr sub_to_dispatch_;
        rclcpp::Publisher<comms::msg::Command>::SharedPtr pub_to_dispatch_;

        std::string locator_topic_ {"/fleet/network"};
        rclcpp::Publisher<comms::msg::NetworkVertex>::SharedPtr pub_to_locator_;
        rclcpp::Subscription<comms::msg::NetworkVertex>::SharedPtr sub_to_locator_;

        std::string formation_topic_ {"/fleet/formation"};
        rclcpp::Subscription<comms::msg::FormationDesired>::SharedPtr sub_to_formation_;
        rclcpp::Publisher<comms::msg::FormationDesired>::SharedPtr pub_to_formation_;

        std::string sync_topic_ {"/fleet/synchronization"};
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_to_sync_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_to_sync_;

        rclcpp::TimerBase::SharedPtr netsize_timer_;
        std::chrono::seconds netsize_timeout_ {constants::ROLLCALL_TIME_SECS};

        std::chrono::seconds rpcs_ack_timeout_ {constants::ACK_TIME_SECS};
};

// Including templates definitions
#include "pelican_templates.tpp"

#endif
