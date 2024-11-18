/**
 * @file roles.cpp
 * @author Simone Vollaro (slim71sv@gmail.com)
 * @brief Methods related to role changing mechanism.
 * @version 1.0.0
 * @date 2024-11-14
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "LoggerModule/logger.hpp"
#include "PelicanModule/pelican.hpp"

/*************************** Role handling ****************************/
/**
 * @brief Execute all operations needed to switch to the Leader role.
 *
 */
void Pelican::becomeLeader() {
    if (this->role_ == leader)
        return;

    this->setRole(leader);
    this->sendLogInfo("Becoming {}", roles_to_string(leader));

    this->hb_core_.flushHeartbeats();
    this->el_core_.flushVotes();

    // Topic preparations
    resetSharedPointer(this->sub_to_sync_);
    this->el_core_.resetSubscriptions();
    this->hb_core_.setupPublisher();
    this->pub_to_sync_ =
        this->create_publisher<comms::msg::FleetSync>(this->sync_topic_, this->qos_);

    // Service preparation
    resetSharedPointer(this->fleetinfo_client_);
    resetSharedPointer(this->form_reached_client_);
    this->teleopdata_server_ = rclcpp_action::create_server<comms::action::TeleopData>(
        this, "contactLeader",
        std::bind(
            &Pelican::handleTeleopDataGoal, this, std::placeholders::_1, std::placeholders::_2
        ),
        std::bind(&Pelican::handleTeleopDataCancellation, this, std::placeholders::_1),
        std::bind(&Pelican::handleAcceptedTeleopData, this, std::placeholders::_1)
    );
    this->fleetinfo_server_ = this->create_service<comms::srv::FleetInfo>(
        "shareinfo_service",
        std::bind(&Pelican::targetNotification, this, std::placeholders::_1, std::placeholders::_2)
    );
    this->form_reached_server_ = this->create_service<comms::srv::FormationReached>(
        "formation_service",
        std::bind(
            &Pelican::recordAgentInFormation, this, std::placeholders::_1, std::placeholders::_2
        ),
        rmw_qos_profile_services_default, this->getReentrantGroup()
    );

    this->hb_core_.sendNow(); // To promptly notify all agents about the new leader
    this->hb_core_.setupTransmissionTimer();
}

/**
 * @brief Execute all operations needed to switch to the Follower role.
 *
 */
void Pelican::becomeFollower() {
    if (this->role_ == follower)
        return;

    this->setRole(follower);
    this->sendLogInfo("Becoming {}", roles_to_string(follower));

    // Topic preparations
    this->commenceStopHeartbeatTransmission();
    this->hb_core_.resetPublisher();
    this->el_core_.prepareTopics();
    resetSharedPointer(this->pub_to_sync_);
    this->sub_to_sync_ = this->create_subscription<comms::msg::FleetSync>(
        this->sync_topic_, this->qos_,
        std::bind(&Pelican::handleSyncCompletedOp, this, std::placeholders::_1)
    );

    // Service preparation
    resetSharedPointer(this->teleopdata_server_);
    resetSharedPointer(this->fleetinfo_server_);
    resetSharedPointer(this->form_reached_server_);
    this->fleetinfo_client_ = this->create_client<comms::srv::FleetInfo>("shareinfo_service");
    this->form_reached_client_ =
        this->create_client<comms::srv::FormationReached>("formation_service");

    this->el_core_.followerActions();
}

/**
 * @brief Execute all operations needed to switch to the Candidate role.
 *
 */
void Pelican::becomeCandidate() {
    if (this->role_ == candidate)
        return;

    this->setRole(candidate);
    this->sendLogInfo("Becoming {}", roles_to_string(candidate));

    // Topic preparations
    this->el_core_.prepareTopics();
    this->commenceStopHeartbeatTransmission();
    this->hb_core_.resetPublisher();
    this->hb_core_.setupSubscription();

    this->el_core_.candidateActions();
}
