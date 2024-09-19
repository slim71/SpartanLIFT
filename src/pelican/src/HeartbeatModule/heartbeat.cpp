#include "HeartbeatModule/heartbeat.hpp"
#include "PelicanModule/pelican.hpp"

/************************** Ctors/Dctors ***************************/
HeartbeatModule::HeartbeatModule() {
    this->node_ = nullptr;
    this->logger_ = nullptr;
}

HeartbeatModule::HeartbeatModule(Pelican* node) : node_(node), logger_ {nullptr} {}

HeartbeatModule::~HeartbeatModule() {
    // Cancel periodic transmission (no problem arises if they're not initialized)
    // Cancel active timers
    cancelTimer(this->hb_transmission_timer_);
    resetSharedPointer(this->hb_transmission_timer_);

    // Clear shared pointers for subscriptions and publishers
    resetSharedPointer(this->sub_to_heartbeat_topic_);
    resetSharedPointer(this->pub_to_heartbeat_topic_);

    // Clear out all heartbeats received and stored
    this->flushHeartbeats();

    this->node_ = nullptr;
    this->logger_ = nullptr;
}

/************************** Setup methods **************************/
void HeartbeatModule::initSetup(LoggerModule* logger) {
    this->setupSubscription();
    resetSharedPointer(this->pub_to_heartbeat_topic_);
    cancelTimer(this->hb_transmission_timer_);

    this->logger_ = logger;
}

void HeartbeatModule::setupPublisher() {
    if (!this->node_) {
        throw MissingExternModule();
    }

    if (!this->pub_to_heartbeat_topic_) {
        this->pub_to_heartbeat_topic_ = this->node_->create_publisher<comms::msg::Heartbeat>(
            this->heartbeat_topic_, this->qos_
        );
    }
}

void HeartbeatModule::setupSubscription() {
    if (!this->node_) {
        throw MissingExternModule();
    }

    // Used by all kinds of agents to avoid multiple leaders
    // (this should not happen, since Raft guarantees safety)
    if (!this->sub_to_heartbeat_topic_) {
        this->sub_to_heartbeat_topic_ = this->node_->create_subscription<comms::msg::Heartbeat>(
            this->heartbeat_topic_, this->qos_,
            std::bind(&HeartbeatModule::storeHeartbeat, this, std::placeholders::_1),
            this->gatherReentrantOptions()
        );
    }
}

void HeartbeatModule::setupTransmissionTimer() {
    if (!this->node_) {
        throw MissingExternModule();
    }

    // Assumed to overwrite the existing one every time, if present.
    this->hb_transmission_timer_ = this->node_->create_wall_timer(
        this->heartbeat_period_, std::bind(&HeartbeatModule::sendHeartbeat, this),
        this->gatherReentrantGroup()
    );
}

/************ Actions initiated from outside the module ************/
void HeartbeatModule::resetPublisher() {
    resetSharedPointer(this->pub_to_heartbeat_topic_);
}

/********************* Special functionalities *********************/
void HeartbeatModule::sendNow() {
    // One-off transmission; allowed for special occasions
    this->sendLogInfo("One-off heartbeat transmission");
    this->sendHeartbeat();
}

/******************** Private member functions *********************/
void HeartbeatModule::sendHeartbeat() {
    comms::msg::Heartbeat hb;
    hb.term_id = this->gatherCurrentTerm();
    hb.leader_id = this->gatherAgentID();
    hb.timestamp = this->gatherTime();

    this->sendLogInfo("Sending heartbeat");

    if (this->pub_to_heartbeat_topic_) {
        this->pub_to_heartbeat_topic_->publish(hb);
    } else {
        this->sendLogError(
            "Publisher to heartbeat topic is not defined! I'm probably not the leader then..."
        );
        cancelTimer(this->hb_transmission_timer_);
    }
}

void HeartbeatModule::stopHeartbeatTransmission() {
    this->sendLogDebug("Stopping heartbeat transmission");
    cancelTimer(this->hb_transmission_timer_);
}

void HeartbeatModule::stopService() {
    this->sendLogWarning("Stopping heartbeat module!");
    cancelTimer(this->hb_transmission_timer_);
    resetSharedPointer(this->pub_to_heartbeat_topic_);
    resetSharedPointer(this->sub_to_heartbeat_topic_);
}

bool HeartbeatModule::checkHeartbeatValidity(const comms::msg::Heartbeat msg) {
    if (msg.leader_id == this->gatherAgentID()) {
        return false;
    }

    if (msg.term_id < this->gatherCurrentTerm()) {
        // Ignore heartbeat; do not reset election timer
        this->sendLogWarning(
            "Ignoring heartbeat from agent {} received with previous term ID ({})", msg.leader_id,
            msg.term_id
        );
        return false;
    }

    return true;
}

void HeartbeatModule::storeHeartbeat(const comms::msg::Heartbeat msg) {
    if (!this->checkHeartbeatValidity(msg))
        return;

    auto term = this->gatherCurrentTerm();

    // Keep heartbeat vector limited
    if (this->getNumberOfHbs() >= this->getMaxHbs()) {
        this->flushHeartbeats();
    }

    // Store heartbeat
    heartbeat hb;
    hb.term = msg.term_id;
    hb.leader = msg.leader_id;
    hb.timestamp = msg.timestamp;

    this->hbs_mutex_.lock();
    this->received_hbs_.push_back(hb);
    // Sort such that the older hb is first, to be sure
    std::sort(
        this->received_hbs_.begin(), this->received_hbs_.end(),
        [](const heartbeat& a, const heartbeat& b) {
            return a.timestamp < b.timestamp;
        }
    );
    this->hbs_mutex_.unlock();

    this->sendLogDebug("Received heartbeat from agent {}", msg.leader_id);

    if (msg.term_id > term) {
        this->sendLogDebug("Aligning term with hearbeat received: {}", msg.term_id);
        this->signalSetTerm(msg.term_id);
    }

    switch (this->gatherAgentRole()) {
        case follower:
            this->signalSetElectionStatus(msg.leader_id);
            this->sendLogInfo("Heartbeat received! Resetting election timer");
            this->signalResetElectionTimer();
            break;
        case candidate:
            this->sendLogWarning("I've received a heartbeat from an external leader");
            this->signalSetElectionStatus(msg.leader_id);
            this->signalTransitionToFollower();
            this->signalResetElectionTimer();
            break;
        case leader:
            if (msg.leader_id != this->gatherAgentID()) {
                this->sendLogWarning(
                    "I've received a heartbeat from some other leader agent ({})", msg.leader_id
                );
                this->signalSetElectionStatus(msg.leader_id);
                this->signalTransitionToFollower();
            }
            break;
        default:
            this->sendLogWarning(
                "Heartbeat received, but my role is undefined! Transitioning to follower"
            );
            this->signalSetElectionStatus(msg.leader_id);
            this->signalTransitionToFollower();
    }
}

void HeartbeatModule::flushHeartbeats() {
    std::lock_guard lock(this->hbs_mutex_);
    this->received_hbs_.clear();
}
