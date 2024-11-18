#include "UNSCModule/unsc.hpp"
#include "PelicanModule/pelican.hpp"

/************************** Ctors/Dctors ***************************/
UNSCModule::UNSCModule() {
    this->node_ = nullptr;
    this->logger_ = nullptr;
}

UNSCModule::UNSCModule(Pelican* node) : node_(node), logger_ {nullptr} {}

UNSCModule::~UNSCModule() {
    // Cancel active timers
    cancelTimer(this->prechecks_timer_);
    resetSharedPointer(this->prechecks_timer_);
    cancelTimer(this->offboard_timer_);
    resetSharedPointer(this->offboard_timer_);
    cancelTimer(this->rendezvous_timer_);
    resetSharedPointer(this->rendezvous_timer_);
    cancelTimer(this->formation_timer_);
    resetSharedPointer(this->formation_timer_);
    cancelTimer(this->linear_timer_);
    resetSharedPointer(this->linear_timer_);
    cancelTimer(this->collision_timer_);
    resetSharedPointer(this->collision_timer_);
    cancelTimer(this->formation_check_timer_);
    resetSharedPointer(this->formation_check_timer_);
    cancelTimer(this->height_check_timer_);
    resetSharedPointer(this->height_check_timer_);
    cancelTimer(this->sync_check_timer_);
    resetSharedPointer(this->sync_check_timer_);
    cancelTimer(this->target_check_timer_);
    resetSharedPointer(this->target_check_timer_);

    neighbors_.clear();
    neigh_des_positions_.clear();
    fleet_order_.clear();

    this->node_ = nullptr;
    this->logger_ = nullptr;
}

/************************** Setup methods **************************/
/**
 * @brief Initialize the UNSCModule object.
 *
 * @param logger RCLCPP logger to link.
 */
void UNSCModule::initSetup(LoggerModule* logger) {
    if (!this->node_) {
        throw MissingExternModule();
    }

    if (!this->logger_)
        this->logger_ = logger;

    // Allow for some loading time before starting all operations...
    this->prechecks_timer_ = this->node_->create_wall_timer(
        this->prechecks_period_, std::bind(&UNSCModule::runPreChecks, this),
        this->gatherReentrantGroup()
    );
}

/**
 * @brief Stop the module's service.
 *
 */
void UNSCModule::stopService() {
    this->sendLogWarning("Stopping UNSC module!");
    cancelTimer(this->prechecks_timer_);
    resetSharedPointer(this->prechecks_timer_);
    cancelTimer(this->offboard_timer_);
    resetSharedPointer(this->offboard_timer_);
    cancelTimer(this->rendezvous_timer_);
    resetSharedPointer(this->rendezvous_timer_);
    cancelTimer(this->formation_timer_);
    resetSharedPointer(this->formation_timer_);
    cancelTimer(this->linear_timer_);
    resetSharedPointer(this->linear_timer_);
    cancelTimer(this->collision_timer_);
    resetSharedPointer(this->collision_timer_);
    cancelTimer(this->formation_check_timer_);
    resetSharedPointer(this->formation_check_timer_);
    cancelTimer(this->height_check_timer_);
    resetSharedPointer(this->height_check_timer_);
    cancelTimer(this->sync_check_timer_);
    resetSharedPointer(this->sync_check_timer_);
    cancelTimer(this->target_check_timer_);
    resetSharedPointer(this->target_check_timer_);
    std::lock_guard lock(this->running_mutex_);
    std::lock_guard lock_cv(this->formation_cv_mutex_);
    this->running_ = false;
    this->neighbor_gathered_ = false;
    this->formation_cv_.notify_all();
}

void UNSCModule::reactToFailureNotification(unsigned int agent) {
    this->sendLogDebug("Handling failure notification from agent {}", agent);

    std::lock_guard lock_closest(this->closest_agent_mutex_);
    if (this->closest_agent_ == agent)
        this->closest_agent_ = 0;

    std::lock_guard lock_neigh(this->neighbors_mutex_);
    auto position = std::find(this->neighbors_.begin(), this->neighbors_.end(), agent);
    if (position != this->neighbors_.end())
        this->neighbors_.erase(position);

    std::lock_guard lock_neighdes(this->neighbors_despos_mutex_);
    this->neigh_des_positions_.erase(agent);

    std::lock_guard lock_order(this->order_mutex_);
    this->fleet_order_.erase(agent);
}

void UNSCModule::heightCompensation(double odom_height) {
    auto maybe_pos_setpoint = this->getPositionSetpoint();
    double target_height = this->getActualTargetHeight();

    if (!this->signalCheckOffboardEngagement() || !maybe_pos_setpoint) {
        this->sendLogDebug(
            "Height compensation skipped: {}",
            maybe_pos_setpoint ? "not in offboard mode" : "the position setpoint is invalid"
        );
        return;
    }

    auto curr_pos_setpoint = maybe_pos_setpoint.value();
    double compensated_height = curr_pos_setpoint.z + (target_height - odom_height);
    this->sendLogDebug(
        "Height| setpoint: {:.4f}, target: {:.4f}, odom: {:.4f}, compensated: {:.4f}",
        curr_pos_setpoint.z, target_height, odom_height, compensated_height
    );

    geometry_msgs::msg::Point vel = NAN_point;
    vel.set__z((compensated_height - curr_pos_setpoint.z) / constants::COMPENSATION_GAP_SECS);

    this->setVelocitySetpoint(vel);
    this->setHeightSetpoint(compensated_height);
}

void UNSCModule::unblockFormation() {
    std::lock_guard lock(this->formation_cv_mutex_);
    this->neighbor_gathered_ = true;
    this->formation_cv_.notify_all();
}

void UNSCModule::waitForOperationCompleted(uint32_t op) {
    if (this->getLastCompletedOperation() >= op) {
        this->sendLogDebug("Operation {} completed!", op);
        cancelTimer(this->sync_check_timer_);
        this->fa_promise_.set_value();
    } else {
        this->sendLogDebug("Operation {} not yet completed", op);
    }
}

bool UNSCModule::safeOrderFind(unsigned int id) {
    std::lock_guard lock(this->order_mutex_);
    if (this->fleet_order_.find(id) == this->fleet_order_.end())
        return true;
    else
        return false;
}

void UNSCModule::setLastCompletedOperation(uint32_t op) {
    std::lock_guard lock(this->last_op_mutex_);
    this->last_op_completed_ = op;
}
