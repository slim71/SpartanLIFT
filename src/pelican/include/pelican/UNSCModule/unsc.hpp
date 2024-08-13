#ifndef __UNSC_HPP__
#define __UNSC_HPP__

#include "LoggerModule/logger.hpp"
#include "types.hpp"
#include "utilities.hpp"

class Pelican;

class UNSCModule {
    public:
        // Ctors/Dctors
        explicit UNSCModule();
        explicit UNSCModule(Pelican*);
        ~UNSCModule();

        // Setup methods
        void initSetup(LoggerModule*);
        void stopService();

        // Simple operations
        void runPreChecks();
        bool arm();
        bool disarm();
        bool takeoff(unsigned int = 0);
        bool land();
        bool setHome();
        bool loiter();
        bool returnToLaunchPosition();
        void activateConsensus();
        void setAndMaintainOffboardMode();
        void heightCompensation(double);

        // Utilities
        void preFormationActions();
        void unblockFormation();
        void increaseSyncCount();
        void decreaseSyncCount();
        void waitForSyncCount();

        // Getters
        bool getRunningStatus() const;
        Eigen::Vector3d getOffset() const;
        std::optional<geometry_msgs::msg::Point> getPositionSetpoint() const;
        std::optional<geometry_msgs::msg::Point> getSetpointVelocity() const;
        double getActualTargetHeight() const;
        std::optional<geometry_msgs::msg::Point> getTargetPosition() const;
        std::vector<unsigned int> getNeighborsIDs() const;
        geometry_msgs::msg::Point getNeighborDesPos(unsigned int);
        unsigned int getClosestAgent() const;
        double getClosestAngle() const;
        unsigned int getFleetOrder(unsigned int) const;
        uint64_t getTargetCount() const;

        // Setters
        void setPositionSetpoint(geometry_msgs::msg::Point);
        void setActualTargetHeight(double);
        void setTargetPosition(geometry_msgs::msg::Point);

    private:
        template<typename... Args> void sendLogInfo(std::string, Args...) const;
        template<typename... Args> void sendLogDebug(std::string, Args...) const;
        template<typename... Args> void sendLogWarning(std::string, Args...) const;
        template<typename... Args> void sendLogError(std::string, Args...) const;

        // PX4-operations runner
        bool sendToCommanderUnit(
            uint16_t, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN,
            float = NAN
        );
        // Algorithms
        geometry_msgs::msg::Point
        adjustmentForCollisionAvoidance(geometry_msgs::msg::Point, double);
        void consensusToRendezvous();
        void formationControl();
        void linearP2P();
        // Neighborhood-related
        void findNeighbors();
        void collectNeighDesPositions(unsigned int);
        void assignFormationPositions();

        // Utilities
        bool safeOrderFind(unsigned int);

        // External communications - getters
        rclcpp::Time gatherTime() const;
        unsigned int gatherAgentID() const;
        double gatherROI() const;
        std::optional<px4_msgs::msg::VehicleStatus> gatherStatus() const;
        unsigned int gatherNetworkSize() const;
        geometry_msgs::msg::Point gatherCopterPosition(unsigned int) const;
        std::optional<nav_msgs::msg::Odometry> gatherENUOdometry() const;
        unsigned int gatherLeaderID() const;
        std::vector<unsigned int> gatherCoptersIDs() const;
        geometry_msgs::msg::Point gatherNeighborDesiredPosition();
        geometry_msgs::msg::Point gatherDesiredPosition() const;
        geometry_msgs::msg::Point gatherDropoffPosition() const;
        // External communications - callback groups
        rclcpp::CallbackGroup::SharedPtr gatherReentrantGroup() const;
        rclcpp::CallbackGroup::SharedPtr gatherOffboardExclusiveGroup() const;
        rclcpp::CallbackGroup::SharedPtr gatherRendezvousExclusiveGroup() const;
        rclcpp::CallbackGroup::SharedPtr gatherFormationExclusiveGroup() const;

        // External communications - starting functionalities
        // command| param1| param2| param3| param4| param5| param6| param7|
        void signalPublishVehicleCommand(
            uint16_t, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN, float = NAN,
            float = NAN
        ) const;
        void signalPublishTrajectorySetpoint(geometry_msgs::msg::Point, geometry_msgs::msg::Point)
            const;
        void signalPublishOffboardControlMode() const;
        bool signalWaitForCommanderAck(uint16_t) const;
        bool signalCheckOffboardEngagement() const;
        void signalSetReferenceHeight(double);
        void signalCargoAttachment();
        void signalSendDesiredFormationPositions(std::unordered_map<
                                                 unsigned int, geometry_msgs::msg::Point>);
        void signalAskDesPosToNeighbor(unsigned int);
        void signalNotifyAgentInFormation();
        void signalSyncTrigger();

        // External communications - flag checks
        bool confirmAgentIsLeader() const;
        bool confirmFormationAchieved() const;

        // Setters
        void setVelocitySetpoint(geometry_msgs::msg::Point, double = 1.0);
        void setHeightSetpoint(double);
        void setClosestAgent(unsigned int);
        void setClosestAngle(double);
        void setFleetOrder(unsigned int, unsigned int);
        void increaseTargetCount();
        void resetTargetCount();
        // Setters - flags
        void unsetNeighborGathered();

    private: // Attributes
        Pelican* node_;
        LoggerModule* logger_;

        // For possible future use
        bool running_ {true};
        bool sitl_ready_ {false};
        bool move_to_center_ {false};
        bool neighbor_gathered_ {false};
        uint64_t offboard_setpoint_counter_ {0}; // counter for the number of setpoints sent
        uint64_t near_target_counter_ {0};       // counter for subsequent setpoints near target
        unsigned int closest_agent_ {0};
        unsigned int sync_count_ {0};
        double actual_target_height_ {0};  // needed in order to stabilize height tracking
        double closest_angle_ {0};
        Eigen::Vector3d offset_ {0, 0, 0}; // [m, m, m]
        geometry_msgs::msg::Point target_position_ = NAN_point; // Actual desired target
        geometry_msgs::msg::Point setpoint_position_ = NAN_point;
        geometry_msgs::msg::Point setpoint_velocity_;
        std::vector<unsigned int> neighbors_;
        std::condition_variable formation_cv_;
        // Map with pair: agent ID - desired position
        std::unordered_map<unsigned int, geometry_msgs::msg::Point> neigh_des_positions_;
        // Map with pair: agent ID - order in the fleet position assignment
        std::unordered_map<unsigned int, unsigned int> fleet_order_;

        mutable std::mutex offset_mutex_;            // to be used with offset_ and yaw_
        mutable std::mutex running_mutex_;           // to be used with running_
        mutable std::mutex setpoint_position_mutex_; // Used to access setpoint_position_
        mutable std::mutex setpoint_velocity_mutex_; // Used to access setpoint_velocity_
        mutable std::mutex height_mutex_;            // Used to access actual_target_height_
        mutable std::mutex target_position_mutex_;   // Used to access target_position_
        mutable std::mutex formation_cv_mutex_;      // Used to access formation_cv_
        mutable std::mutex neighbors_mutex_;         // Used to access neighbors_
        mutable std::mutex neighbors_despos_mutex_;  // Used to access neigh_des_positions_
        mutable std::mutex closest_agent_mutex_;     // Used to access closest_agent_
        mutable std::mutex closest_angle_mutex_;     // Used to access closest_angle_
        mutable std::mutex sync_mutex_;              // Used to access sync_count_
        mutable std::mutex order_mutex_;             // Used to access fleet_order_
        mutable std::mutex target_count_mutex_;      // Used to access near_target_counter_

        rclcpp::TimerBase::SharedPtr prechecks_timer_;
        std::chrono::seconds prechecks_period_ {constants::PRECHECKS_TIME_SECS};

        rclcpp::TimerBase::SharedPtr offboard_timer_;
        std::chrono::milliseconds offboard_period_ {constants::OFFBOARD_PERIOD_MILLIS};

        rclcpp::TimerBase::SharedPtr rendezvous_timer_;
        std::chrono::milliseconds rendezvous_period_ {
            std::chrono::milliseconds(constants::RENDEZVOUS_PERIOD_MILLIS)};

        rclcpp::TimerBase::SharedPtr formation_timer_;
        std::chrono::milliseconds formation_period_ {
            std::chrono::milliseconds(constants::FORMATION_PERIOD_MILLIS)};

        rclcpp::TimerBase::SharedPtr linear_timer_;
        std::chrono::milliseconds linear_period_ {
            std::chrono::milliseconds(constants::P2P_PERIOD_MILLIS)};
};

#include "unsc_template.tpp"

#endif
