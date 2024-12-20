#include "TacMapModule/tacmap.hpp"
#include "PelicanModule/pelican.hpp"

/************************** Ctors/Dctors ***************************/
TacMapModule::TacMapModule() {
    this->node_ = nullptr;
    this->logger_ = nullptr;
}

TacMapModule::TacMapModule(Pelican* node) : node_(node), logger_ {nullptr} {}

TacMapModule::~TacMapModule() {
    // Cancel active timers
    cancelTimer(position_timer_);
    resetSharedPointer(position_timer_);
    cancelTimer(compensation_timer_);
    resetSharedPointer(compensation_timer_);

    // Cancel and reset all subscriptions and publishers
    resetSharedPointer(this->sub_to_flags_topic_);
    resetSharedPointer(this->sub_to_attitude_topic_);
    resetSharedPointer(this->sub_to_control_mode_topic_);
    resetSharedPointer(this->sub_to_status_topic_);
    resetSharedPointer(this->sub_to_command_ack_topic_);
    resetSharedPointer(this->sub_to_enu_odometry_topic_);
    resetSharedPointer(this->pub_to_command_topic_);
    resetSharedPointer(this->pub_to_trajectory_setpoint_topic_);
    resetSharedPointer(this->pub_to_offboard_control_topic_);

    last_commander_ack_.reset();
    enu_odometry_buffer_.clear();
    status_buffer_.clear();

    this->node_ = nullptr;
    this->logger_ = nullptr;
}

/************************** Setup methods **************************/
void TacMapModule::initTopics() {
    std::string px4_header;

    try {
        px4_header = "/px4_"s + std::to_string(this->gatherAgentID());
    } catch (const std::exception& exc) {
        this->sendLogWarning("No main module found!");
        px4_header = "px4_1"s;
    }

    this->flags_topic_ = px4_header + "/fmu/out/failsafe_flags"s;
    this->attitude_topic_ = px4_header + "/fmu/out/vehicle_attitude"s;
    this->control_mode_topic_ = px4_header + "/fmu/out/vehicle_control_mode"s;
    this->command_ack_topic_ = px4_header + "/fmu/out/vehicle_command_ack"s;
    this->status_topic_ = px4_header + "/fmu/out/vehicle_status"s;

    this->command_topic_ = px4_header + "/fmu/in/vehicle_command"s;
    this->trajectory_setpoint_topic_ = px4_header + "/fmu/in/trajectory_setpoint"s;
    this->offboard_control_topic_ = px4_header + "/fmu/in/offboard_control_mode"s;

    std::string model = this->gatherAgentModel();
    unsigned int last_slash = model.find_last_of("/");
    std::string model_folder = model.substr(0, last_slash);
    unsigned int second_last_slash = model_folder.find_last_of("/");
    std::string model_name =
        model.substr(second_last_slash + 1, last_slash - second_last_slash - 1);
    this->enu_odometry_topic_ =
        "/model/" + model_name + "_" + std::to_string(this->gatherAgentID()) + "/odometry";
}

void TacMapModule::initSubscribers() {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->sub_to_status_topic_ = this->node_->create_subscription<px4_msgs::msg::VehicleStatus>(
        this->status_topic_, this->px4_qos_,
        std::bind(&TacMapModule::storeStatus, this, std::placeholders::_1),
        this->gatherReentrantOptions()
    );

    this->sub_to_command_ack_topic_ =
        this->node_->create_subscription<px4_msgs::msg::VehicleCommandAck>(
            this->command_ack_topic_, this->px4_qos_,
            std::bind(&TacMapModule::storeAck, this, std::placeholders::_1),
            this->gatherReentrantOptions()
        );

    this->sub_to_enu_odometry_topic_ = this->node_->create_subscription<nav_msgs::msg::Odometry>(
        this->enu_odometry_topic_, this->standard_qos_,
        std::bind(&TacMapModule::storeENUOdometry, this, std::placeholders::_1),
        this->gatherReentrantOptions()
    );
}

void TacMapModule::initPublishers() {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->pub_to_command_topic_ = this->node_->create_publisher<px4_msgs::msg::VehicleCommand>(
        this->command_topic_, this->standard_qos_
    );

    this->pub_to_offboard_control_topic_ =
        this->node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
            this->offboard_control_topic_, this->standard_qos_
        );

    this->pub_to_trajectory_setpoint_topic_ =
        this->node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            this->trajectory_setpoint_topic_, this->standard_qos_
        );
}

/**
 * @brief Initialize the TacMapModule object.
 *
 * @param logger RCLCPP logger to link.
 */
void TacMapModule::initSetup(LoggerModule* logger) {
    if (!this->logger_)
        this->logger_ = logger;

    this->initTopics();
    this->initSubscribers();
    this->initPublishers();

    // Set a timer to constantly share own position with the other agents
    this->position_timer_ = this->node_->create_wall_timer(
        std::chrono::milliseconds(constants::POS_SHARING_PERIOD_MILLIS),
        [this]() {
            auto maybe_odom = this->getENUOdometry();
            if (!maybe_odom)
                return;
            auto last_enu_odom = maybe_odom.value();
            this->signalSharePosition(last_enu_odom.pose.pose.position);
        },
        this->gatherReentrantGroup()
    );

    // Set a timer for height tracking compensation
    this->compensation_timer_ = this->node_->create_wall_timer(
        std::chrono::seconds(constants::COMPENSATION_GAP_SECS),
        std::bind(&TacMapModule::triggerHeightTracking, this), this->gatherReentrantGroup()
    );
}

/**
 * @brief Stop the module's service.
 *
 */
void TacMapModule::stopService() {
    this->sendLogWarning("Stopping TacMap module!");
    cancelTimer(this->position_timer_);
    cancelTimer(this->compensation_timer_);
    resetSharedPointer(this->sub_to_flags_topic_);
    resetSharedPointer(this->sub_to_attitude_topic_);
    resetSharedPointer(this->sub_to_control_mode_topic_);
    resetSharedPointer(this->sub_to_status_topic_);
    resetSharedPointer(this->sub_to_command_ack_topic_);
    resetSharedPointer(this->sub_to_enu_odometry_topic_);
    resetSharedPointer(this->pub_to_command_topic_);
    resetSharedPointer(this->pub_to_trajectory_setpoint_topic_);
    resetSharedPointer(this->pub_to_offboard_control_topic_);
    std::lock_guard lock(this->running_mutex_);
    this->running_ = false;
}

bool TacMapModule::checkOffboardEngagement() {
    auto last_status = this->getStatus();

    if (last_status &&
        (last_status->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) &&
        (last_status->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)) {
        return true;
    }

    return false;
}

void TacMapModule::triggerHeightTracking() {
    auto maybe_odom = this->getENUOdometry();
    if (!maybe_odom)
        return;
    auto last_enu_odom = maybe_odom.value();
    this->sendLogDebug("Height from global odometry: {}", last_enu_odom.pose.pose.position.z);
    this->signalHeightCompensation(last_enu_odom.pose.pose.position.z);
}
