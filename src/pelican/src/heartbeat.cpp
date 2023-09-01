#include "heartbeat.hpp"
#include "pelican.hpp"

HeartbeatModule::HeartbeatModule(Pelican* node) : node_(node) {
    // Not random, it has to be lower than the election_timeout_
    this->heartbeat_period_ = std::chrono::milliseconds(100);

    // Heartbeat period is 100ms, so this keeps a log of 10s
    this->max_hbs_ = 100;
}

HeartbeatModule::~HeartbeatModule() {
    // Cancel periodic transmission (no problem arises if they're not initialized)
    cancelTimer(this->hb_transmission_timer_);
    // Delete subscriber and publisher
    this->resetSubscription();
    resetSharedPointer(this->pub_to_heartbeat_topic_);
    // Clear out all heartbeats received and stored
    this->flushHeartbeats();
}

void HeartbeatModule::initSetup(LoggerModule* logger) {
    /******************* Subscribrers ****************************/
    // Used by all kinds of agents to avoid multiple leaders 
    // (this should not happen, since Raft guarantees safety)
    if (!this->sub_to_heartbeat_topic_) {
        this->sub_to_heartbeat_topic_ = this->node_->create_subscription<comms::msg::Heartbeat>(
            this->heartbeat_topic_, this->qos_,
            std::bind(&HeartbeatModule::storeHeartbeat, this, std::placeholders::_1),
            this->node_->getReentrantOptions()
        );
    }

    /******************* Publishers ****************************/
    resetSharedPointer(this->pub_to_heartbeat_topic_);

    /*********************** Timers ****************************/
    cancelTimer(this->hb_transmission_timer_);

    this->logger_ = logger;
}

void HeartbeatModule::setupPublisher() {
    if(!this->pub_to_heartbeat_topic_) {
        this->pub_to_heartbeat_topic_ =
            this->node_->create_publisher<comms::msg::Heartbeat>(this->heartbeat_topic_, this->qos_);
    }
}

void HeartbeatModule::resetSubscription() {
    resetSharedPointer(this->sub_to_heartbeat_topic_);
}

void HeartbeatModule::flushStorage() {
    this->flushHeartbeats();
}

void HeartbeatModule::setTransmissionTimer() {
    // Assumed to overwrite the existing one every time, if present
    this->hb_transmission_timer_ = this->node_->create_wall_timer(
        this->heartbeat_period_, std::bind(&HeartbeatModule::sendHeartbeat, this),
        this->node_->getReentrantGroup()
    );
}

void HeartbeatModule::sendNow() {
    // One-off transmission allowed for special cases
    this->logger_->logInfo("One-off heartbeat transmission", hb_module);
    this->sendHeartbeat();
}

void HeartbeatModule::sendHeartbeat() const {
    this->logger_->logInfo("Sending heartbeat", hb_module);

    comms::msg::Heartbeat hb;
    hb.term_id = this->node_->getCurrentTerm();
    hb.leader_id = this->node_->getID();
    hb.timestamp = this->node_->now();

    if (this->pub_to_heartbeat_topic_) {
        this->pub_to_heartbeat_topic_->publish(hb);
    } else {
        this->logger_->logError("Publisher to heartbeat topic is not defined!", hb_module);
    }
}

void HeartbeatModule::stopHeartbeat() {
    this->logger_->logInfo("Stopping heartbeat transmissions", hb_module);
    cancelTimer(this->hb_transmission_timer_);
}

void HeartbeatModule::storeHeartbeat(const comms::msg::Heartbeat msg) {
    if (msg.term_id < this->node_->getCurrentTerm()) {
        // Ignore heartbeat
        this->logger_->logWarning(
            "Ignoring heartbeat received with previous term ID ({} vs {})", hb_module, msg.term_id,
            this->node_->getCurrentTerm()
        );
        return;
    }

    // If it comes to this, the msg.term is at least equal to the node's current term

    // For any node; this should not apply to leaders
    this->node_->setElectionStatus(msg.leader_id);

    this->logger_->logDebug("Resetting election_timer_...", hb_module);
    this->node_->resetElectionTimer();

    // Keep heartbeat vector limited
    if (this->getNumberOfHbs() >= this->getMaxHbs()) {
        this->flushHeartbeats();
    }

    heartbeat hb;
    hb.term = msg.term_id;
    hb.leader = msg.leader_id;
    hb.timestamp = msg.timestamp;

    this->logger_->logInfo("Received heartbeat from agent {} during term {}", hb_module, 
                    msg.leader_id, msg.term_id);

    this->hbs_mutex_.lock();
    this->received_hbs_.push_back(hb);
    // Sort such that older hb is first, to be sure
    std::sort(
        this->received_hbs_.begin(), this->received_hbs_.end(),
        [](const heartbeat& a, const heartbeat& b) { return a.timestamp < b.timestamp; }
    );
    this->hbs_mutex_.unlock();

    if (this->node_->getRole() == leader) { // Switch back to follower!
        // This should never be needed, since Raft guarantees safety
        this->logger_->logWarning("As a leader, I've received a heartbeat from some other leader agent", hb_module);
        this->node_->commenceFollowerOperations();
    }
}

void HeartbeatModule::flushHeartbeats() {
    std::lock_guard<std::mutex> lock(this->hbs_mutex_);
    this->received_hbs_.clear();
}
