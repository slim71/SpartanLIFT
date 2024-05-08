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
        double getCollisionRadius() const;
        unsigned int getCurrentTerm() const;
        rclcpp::Time getTime() const;
        unsigned int getNetworkSize() const;
        std::optional<geometry_msgs::msg::Point> getSetpointPosition() const;
        std::optional<geometry_msgs::msg::Point> getSetpointVelocity() const;
        std::optional<geometry_msgs::msg::Point> getTargetPosition() const;
        double getActualTargetHeight() const;
        geometry_msgs::msg::Point getCopterPosition(unsigned int) const;

        // For callback groups
        rclcpp::SubscriptionOptions getReentrantOptions() const;
        rclcpp::CallbackGroup::SharedPtr getReentrantGroup() const;
        // Each callback in these groups should not execute in parallel with itself,
        // but can be parallel to one another
        rclcpp::CallbackGroup::SharedPtr getTimerExclusiveGroup() const;
        rclcpp::CallbackGroup::SharedPtr getOffboardExclusiveGroup() const;
        rclcpp::CallbackGroup::SharedPtr getRendezvousExclusiveGroup() const;

        // Actions initiated by the module
        bool initiateSetHome();
        bool initiateArm();
        bool initiateTakeoff();
        bool initiateLand();
        bool initiateReturnToLaunchPosition();
        void initiateOffboardMode();

        // Status check
        bool isLeader() const;
        bool isFollower() const;
        bool isCandidate() const;
        bool isReady() const;
        bool isFlying() const;
        bool isCarrying() const;
        bool isLastCmdExecuted() const;

        // Handle data exchange among modules
        heartbeat requestLastHb();
        int requestNumberOfHbs();
        std::optional<px4_msgs::msg::VehicleOdometry> requestNEDOdometry();
        std::optional<nav_msgs::msg::Odometry> requestENUOdometry();
        std::optional<px4_msgs::msg::VehicleStatus> requestStatus();

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
        );                                                           // UNSC module
        void commenceSetSetpointPosition(geometry_msgs::msg::Point); // UNSC module
        void commenceSetSetpointVelocity(geometry_msgs::msg::Point); // UNSC module
        bool commenceCheckOffboardEngagement();                      // TacMap and UNSC modules
        bool commenceWaitForCommanderAck(uint16_t);                  // TacMap module
        void commenceHeightCompensation(double);                     // TacMap module
        void commenceSharePosition(geometry_msgs::msg::Point);       // TacMap module

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
        void storeAttendance(comms::msg::NetworkVertex::SharedPtr);
        void storeCopterInfo(const comms::msg::NetworkVertex::SharedPtr msg);

        void becomeLeader();
        void becomeFollower();
        void becomeCandidate();

        void resizeCopterPositionsVector(unsigned int, unsigned int = UINT_MAX);

        void setID(unsigned int);
        void setMass(double);
        void setRole(possible_roles);
        void setTerm(unsigned int);
        void setSetpointPosition(geometry_msgs::msg::Point);
        void setSetpointVelocity(geometry_msgs::msg::Point);
        void setTargetPosition(geometry_msgs::msg::Point);
        void setReferenceHeight(double);
        void setFlyingStatus();
        void unsetFlyingStatus();
        void setCarryingStatus();
        void unsetCarryingStatus();
        void setLastCmdStatus();
        void unsetLastCmdStatus();
        void setAndNotifyRendezvousHandled();
        void unsetAndNotifyRendezvousHandled();

        void rogerWillCo(const std::shared_ptr<
                         rclcpp_action::ServerGoalHandle<comms::action::TeleopData>>);
        rclcpp_action::GoalResponse
        handleTeleopDataGoal(const rclcpp_action::GoalUUID&, std::shared_ptr<const comms::action::TeleopData::Goal>);
        rclcpp_action::CancelResponse
        handleTeleopDataCancellation(const std::shared_ptr<
                                     rclcpp_action::ServerGoalHandle<comms::action::TeleopData>>);
        void handleAcceptedTeleopData(const std::shared_ptr<
                                      rclcpp_action::ServerGoalHandle<comms::action::TeleopData>>);

        bool checkCommandMsgValidity(const comms::msg::Command);
        void handleCommandDispatch(uint16_t);
        void handleCommandReception(const comms::msg::Command);
        bool broadcastCommand(uint16_t);
        void sendAppendEntryRPC(unsigned int, uint16_t, bool = false, bool = false);
        void sendRPCAck(unsigned int, uint16_t, bool = false);
        bool waitForRPCsAcks(uint16_t, bool = false);
        void appendEntry(uint16_t, unsigned int);
        bool executeRPCCommand(uint16_t);
        void rendezvousFleet();
        void sharePosition(geometry_msgs::msg::Point);
        void recordCopterPosition(comms::msg::NetworkVertex::SharedPtr);
        unsigned int initiateGetLeaderID();

        void
        targetNotification(const std::shared_ptr<comms::srv::FleetInfo::Request>, const std::shared_ptr<comms::srv::FleetInfo::Response>);
        void processLeaderResponse(rclcpp::Client<comms::srv::FleetInfo>::SharedFuture);

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
        double roi_;
        double collision_radius_;
        bool ready_ {false};
        bool flying_ {false};
        bool carrying_ {false};
        bool mission_in_progress_ {false};
        bool last_cmd_result_ {false};
        std::string model_;
        possible_roles role_ {tbd};
        double actual_target_height_ {0}; // needed in order to stabilize height tracking
        geometry_msgs::msg::Point setpoint_position_ =
            NAN_point;                    // referring to temporary setpoint along a trajectory
        geometry_msgs::msg::Point
            setpoint_velocity_;           // referring to temporary setpoint along a trajectory
        geometry_msgs::msg::Point target_position_ = NAN_point; // Actual desired target
        std::vector<geometry_msgs::msg::Point> copters_positions_;
        std::vector<std::tuple<unsigned int, unsigned int>> rpcs_vector_;
        std::vector<comms::msg::NetworkVertex> discovery_vector_;
        std::vector<comms::msg::Command> dispatch_vector_;
        std::shared_ptr<rclcpp_action::ServerGoalHandle<comms::action::TeleopData>>
            last_goal_handle_;
        TriState rendezvous_handled_ {TriState::Floating};
        std::condition_variable rend_handled_cv_;

        mutable std::mutex id_mutex_;                // Used to access id_
        mutable std::mutex term_mutex_;              // Used to access current_term_
        mutable std::mutex flying_mutex_;            // Used to access flying_
        mutable std::mutex carrying_mutex_;          // Used to access carrying_
        mutable std::mutex setpoint_position_mutex_; // Used to access setpoint_position_
        mutable std::mutex setpoint_velocity_mutex_; // Used to access setpoint_velocity_
        mutable std::mutex target_position_mutex_;   // Used to access target_position_
        mutable std::mutex height_mutex_;            // Used to access actual_target_height_
        mutable std::mutex positions_mutex_;         // to be used with copters_positions_
        mutable std::mutex discovery_mutex_;         // To be used with discovery_vector_
        mutable std::mutex dispatch_mutex_;          // To be used with dispatch_vector_
        mutable std::mutex rpcs_mutex_;              // To be used with rpcs_vector_
        mutable std::mutex last_cmd_result_mutex_;   // To be used with last_cmd_result_
        mutable std::mutex last_goal_mutex_;         // To be used with last_goal_handle_
        mutable std::mutex rendez_tristate_mutex_;   // to be used with rendezvous_handled_

        rclcpp::SubscriptionOptions reentrant_opt_ {rclcpp::SubscriptionOptions()};
        rclcpp::CallbackGroup::SharedPtr reentrant_group_;
        rclcpp::SubscriptionOptions timer_exclusive_opt_ {rclcpp::SubscriptionOptions()};
        rclcpp::CallbackGroup::SharedPtr timer_exclusive_group_;
        rclcpp::SubscriptionOptions offboard_exclusive_opt_ {rclcpp::SubscriptionOptions()};
        rclcpp::CallbackGroup::SharedPtr offboard_exclusive_group_;
        rclcpp::SubscriptionOptions rendezvous_exclusive_opt_ {rclcpp::SubscriptionOptions()};
        rclcpp::CallbackGroup::SharedPtr rendezvous_exclusive_group_;

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

        std::string dispatch_topic_ {"/fleet/dispatch"};
        rclcpp::Subscription<comms::msg::Command>::SharedPtr sub_to_dispatch_;
        rclcpp::Publisher<comms::msg::Command>::SharedPtr pub_to_dispatch_;

        std::string locator_topic_ {"/fleet/network"};
        rclcpp::Publisher<comms::msg::NetworkVertex>::SharedPtr pub_to_locator_;
        rclcpp::Subscription<comms::msg::NetworkVertex>::SharedPtr sub_to_locator_;

        rclcpp::TimerBase::SharedPtr netsize_timer_;
        std::chrono::seconds netsize_timeout_ {constants::ROLLCALL_TIME_SECS};

        std::chrono::seconds rpcs_ack_timeout_ {constants::ACK_TIME_SECS};
};

// Including templates definitions
#include "pelican_templates.tpp"

#endif
