#include "TacMapModule/tacmap.hpp"
#include "PelicanModule/pelican.hpp"

/************************** Ctors/Dctors ***************************/
TacMapModule::TacMapModule() {
    this->node_ = nullptr;
    this->logger_ = nullptr;
}

TacMapModule::TacMapModule(Pelican* node) : node_(node), logger_ {nullptr} {}

TacMapModule::~TacMapModule() {
    /* ... */

    // Unsubscribe from topics
    this->sub_to_flags_topic_.reset();
    this->sub_to_attitude_topic_.reset();
    this->sub_to_control_mode_topic_.reset();
    this->sub_to_global_pos_topic_.reset();
    this->sub_to_gps_pos_topic_.reset();
    this->sub_to_local_pos_topic_.reset();
    this->sub_to_odometry_topic_.reset();
    this->sub_to_status_topic_.reset();

    this->node_ = nullptr;
    this->logger_ = nullptr;
}

/************************** Setup methods **************************/
void TacMapModule::initTopics() {
    std::string px4_header;

    try {
        px4_header = "/px4_"s + std::to_string(this->gatherAgentID());
    } catch (const std::exception& exc) {
        this->sendLogError("TacMapModule initTopics failed!");
        // Abort everything
        throw std::runtime_error("TacMapModule initTopics failed!");
    }

    this->flags_topic_ = px4_header + "/fmu/out/failsafe_flags"s;
    this->attitude_topic_ = px4_header + "/fmu/out/vehicle_attitude"s;
    this->control_mode_topic_ = px4_header + "/fmu/out/vehicle_control_mode"s;
    this->global_pos_topic_ = px4_header + "/fmu/out/vehicle_global_position"s;
    this->gps_pos_topic_ = px4_header + "/fmu/out/vehicle_gps_position"s;
    this->local_pos_topic_ = px4_header + "/fmu/out/vehicle_local_position"s;
    this->odometry_topic_ = px4_header + "/fmu/out/vehicle_odometry"s;
    this->command_ack_topic_ = px4_header + "/fmu/out/vehicle_command_ack"s;
    this->status_topic_ = px4_header + "/fmu/out/vehicle_status"s;

    this->command_topic_ = px4_header + "/fmu/in/vehicle_command"s;
    this->offboard_control_topic_ = px4_header + "/fmu/in/offboard_control_mode"s;
    this->trajectory_setpoint_topic_ = px4_header + "/fmu/in/trajectory_setpoint"s;
}

void TacMapModule::initSubscribers() {
    // this->sub_to_flags_topic_ = this->node_->create_subscription<px4_msgs::msg::FailsafeFlags>(
    //     this->flags_topic_, this->px4_qos_,
    //     std::bind(
    //         static_cast<void (TacMapModule::*)(const px4_msgs::msg::FailsafeFlags::SharedPtr)
    //                         const>(&TacMapModule::printData),
    //         this, std::placeholders::_1
    //     ),
    //     this->gatherReentrantOptions()
    // );

    // this->sub_to_attitude_topic_ =
    // this->node_->create_subscription<px4_msgs::msg::VehicleAttitude>(
    //     this->attitude_topic_, this->px4_qos_,
    //     std::bind(
    //         static_cast<void (TacMapModule::*)(const px4_msgs::msg::VehicleAttitude::SharedPtr)
    //                         const>(&TacMapModule::printData),
    //         this, std::placeholders::_1
    //     ),
    //     this->gatherReentrantOptions()
    // );

    // this->sub_to_control_mode_topic_ = this->node_->create_subscription<
    //     px4_msgs::msg::VehicleControlMode>(
    //     this->control_mode_topic_, this->px4_qos_,
    //     std::bind(
    //         static_cast<void (TacMapModule::*)(const
    //         px4_msgs::msg::VehicleControlMode::SharedPtr)
    //                         const>(&TacMapModule::printData),
    //         this, std::placeholders::_1
    //     ),
    //     this->gatherReentrantOptions()
    // );

    this->sub_to_global_pos_topic_ =
        this->node_->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
            this->global_pos_topic_, this->px4_qos_,
            std::bind(&TacMapModule::storeGlobalPosition, this, std::placeholders::_1),
            this->gatherReentrantOptions()
        );

    // this->sub_to_gps_pos_topic_ = this->node_->create_subscription<px4_msgs::msg::SensorGps>(
    //     this->gps_pos_topic_, this->px4_qos_,
    //     std::bind(&TacMapModule::storeGps, this, std::placeholders::_1),
    //     this->gatherReentrantOptions()
    // );

    // // In NED. The coordinate system origin is the vehicle position at the time when the
    // EKF2-module
    // // was started. Needed by every type of agent and never canceled
    // this->sub_to_local_pos_topic_ = this->node_->create_subscription<
    //     px4_msgs::msg::VehicleLocalPosition>(
    //     this->local_pos_topic_, this->px4_qos_,
    //     std::bind(
    //         static_cast<void (TacMapModule::*)(const
    //         px4_msgs::msg::VehicleLocalPosition::SharedPtr)
    //                         const>(&TacMapModule::printData),
    //         this, std::placeholders::_1
    //     ),
    //     this->gatherReentrantOptions()
    // );

    this->sub_to_odometry_topic_ = this->node_->create_subscription<px4_msgs::msg::VehicleOdometry>(
        this->odometry_topic_, this->px4_qos_,
        std::bind(&TacMapModule::storeOdometry, this, std::placeholders::_1),
        this->gatherReentrantOptions()
    );

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
}

void TacMapModule::initPublishers() {
    this->pub_to_command_topic_ = this->node_->create_publisher<px4_msgs::msg::VehicleCommand>(
        this->command_topic_, this->standard_qos_value_
    );

    this->pub_to_offboard_control_topic_ =
        this->node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
            this->offboard_control_topic_, this->standard_qos_value_
        );

    this->pub_to_trajectory_setpoint_topic =
        this->node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            this->trajectory_setpoint_topic_, this->standard_qos_value_
        );
}

void TacMapModule::initSetup(LoggerModule* logger) {
    this->logger_ = logger;

    this->initTopics();
    this->initSubscribers();
    this->initPublishers();

    // this->offboard_timer_ = this->node_->create_wall_timer(
    //     this->offboard_period_, std::bind(&TacMapModule::offboardTimerCallback, this)
    // );
    this->offboard_timer_ = this->node_->create_wall_timer(
        std::chrono::milliseconds(10000), std::bind(&TacMapModule::takeoff, this)
    );
}

void TacMapModule::stopData() {
    std::lock_guard<std::mutex> lock(this->running_mutex_);
    this->running_ = false;
}

bool TacMapModule::checkIsRunning() {
    std::lock_guard<std::mutex> lock(this->running_mutex_);
    return this->running_;
}
