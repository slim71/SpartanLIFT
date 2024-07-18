#include "UNSCModule/unsc.hpp"
#include "PelicanModule/pelican.hpp"

/************************** Ctors/Dctors ***************************/
UNSCModule::UNSCModule() {
    this->node_ = nullptr;
    this->logger_ = nullptr;
}

UNSCModule::UNSCModule(Pelican* node) : node_(node), logger_ {nullptr} {}

UNSCModule::~UNSCModule() {
    this->node_ = nullptr;
    this->logger_ = nullptr;
}

/************************** Setup methods **************************/

void UNSCModule::initSetup(LoggerModule* logger) {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->logger_ = logger;

    // Allow for some loading time before starting all operations...
    this->prechecks_timer_ = this->node_->create_wall_timer(
        this->prechecks_period_, std::bind(&UNSCModule::runPreChecks, this),
        this->gatherReentrantGroup()
    );
}

void UNSCModule::stopService() {
    this->sendLogDebug("Stopping unsc module");
    std::lock_guard lock(this->running_mutex_);
    this->running_ = false;
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
