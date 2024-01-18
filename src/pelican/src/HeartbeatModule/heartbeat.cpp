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
    cancelTimer(this->hb_transmission_timer_);

    // Clear out all heartbeats received and stored
    this->flushHeartbeats();

    this->node_ = nullptr;
    this->logger_ = nullptr;
}

/************************** Setup methods **************************/
void HeartbeatModule::initSetup(LoggerModule* logger) {
    /******************* Subscribrers ****************************/
    this->setupSubscription();

    /******************* Publishers ****************************/
    resetSharedPointer(this->pub_to_heartbeat_topic_);

    /*********************** Timers ****************************/
    cancelTimer(this->hb_transmission_timer_);

    this->logger_ = logger;
}

void HeartbeatModule::setupPublisher() {
    if (!this->node_) {
        throw MissingExternModule();
    }

    // If no error has been thrown, node_ is actually set and this can be executed
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
        // If no error has been thrown, node_ is actually set and this can be executed
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
    // If no error has been thrown, node_ is actually set and this can be executed
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
void HeartbeatModule::sendHeartbeat() const {
    // In case of missing MainModule, we could consider sending the ERROR_HB,
    // but to no use. Let the module just throw an exception

    comms::msg::Heartbeat hb;
    hb.term_id = this->gatherCurrentTerm();
    hb.leader_id = this->gatherAgentID();
    hb.timestamp = this->gatherTime();

    this->sendLogInfo("Sending heartbeat (term {})", hb.term_id);

    if (this->pub_to_heartbeat_topic_) {
        this->pub_to_heartbeat_topic_->publish(hb);
    } else {
        this->sendLogError("Publisher to heartbeat topic is not defined!");
    }
}

void HeartbeatModule::stopService() {
    if (this->hb_transmission_timer_) {
        this->sendLogInfo("Stopping heartbeat transmissions");
    }
    cancelTimer(this->hb_transmission_timer_);
}

void HeartbeatModule::storeHeartbeat(const comms::msg::Heartbeat msg) {
    auto term = this->gatherCurrentTerm();

    if (msg.leader_id == this->gatherAgentID()) {
        return;
    }

    if (msg.term_id < term) {
        // Ignore heartbeat; do not reset election timer
        this->sendLogWarning(
            "Ignoring heartbeat form agent received with previous term ID ({} vs {})",
            msg.leader_id, msg.term_id, term
        );
        return;
    }

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
    // Sort such that older hb is first, to be sure
    std::sort(
        this->received_hbs_.begin(), this->received_hbs_.end(),
        [](const heartbeat& a, const heartbeat& b) {
            return a.timestamp < b.timestamp;
        }
    );
    this->hbs_mutex_.unlock();

    this->sendLogDebug(
        "Received heartbeat from agent {} during term {}", msg.leader_id, msg.term_id
    );

    switch (this->gatherAgentRole()) {
        case follower:
            if (msg.term_id > term)
                this->signalSetTerm(msg.term_id); // CHECK: maybe directly set msg.term_id
            this->signalSetElectionStatus(msg.leader_id);
            this->sendLogInfo("Heartbeat received! Resetting election timer");
            this->signalResetElectionTimer();
            break;
        case candidate:
            this->sendLogWarning("As a candidate, I've received a heartbeat from an external leader"
            );
            this->signalSetElectionStatus(msg.leader_id);
            this->signalTransitionToFollower();
            break;
        case leader:
            if (msg.leader_id != this->gatherAgentID()) {
                this->sendLogWarning(
                    "As a leader, I've received a heartbeat from some other leader agent"
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
    std::lock_guard<std::mutex> lock(this->hbs_mutex_);
    this->received_hbs_.clear();
}
