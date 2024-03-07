#include "TacMapModule/tacmap.hpp"
#include "PelicanModule/pelican.hpp"

/************************** Ctors/Dctors ***************************/
TacMapModule::TacMapModule() {
    this->node_ = nullptr;
    this->logger_ = nullptr;
}

TacMapModule::TacMapModule(Pelican* node) : node_(node), logger_ {nullptr} {}

TacMapModule::~TacMapModule() {
    // Unsubscribe from topics
    resetSharedPointer(this->sub_to_flags_topic_);
    resetSharedPointer(this->sub_to_attitude_topic_);
    resetSharedPointer(this->sub_to_control_mode_topic_);
    resetSharedPointer(this->sub_to_global_pos_topic_);
    resetSharedPointer(this->sub_to_odometry_topic_);
    resetSharedPointer(this->sub_to_status_topic_);

    // for possible future use
    // resetSharedPointer(this->sub_to_local_pos_topic_);

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
    this->global_pos_topic_ = px4_header + "/fmu/out/vehicle_global_position"s;
    this->odometry_topic_ = px4_header + "/fmu/out/vehicle_odometry"s;
    this->command_ack_topic_ = px4_header + "/fmu/out/vehicle_command_ack"s;
    this->status_topic_ = px4_header + "/fmu/out/vehicle_status"s;

    this->command_topic_ = px4_header + "/fmu/in/vehicle_command"s;
    this->trajectory_setpoint_topic_ = px4_header + "/fmu/in/trajectory_setpoint"s;
    this->offboard_control_topic_ = px4_header + "/fmu/in/offboard_control_mode"s;

    /* for possible future use
        this->local_pos_topic_ = px4_header + "/fmu/out/vehicle_local_position"s;
        std::string model = this->gatherAgentModel();
        unsigned int last_slash = model.find_last_of("/");
        std::string model_folder = model.substr(0, last_slash);
        unsigned int second_last_slash = model_folder.find_last_of("/");
        std::string model_name =
            model.substr(second_last_slash + 1, last_slash - second_last_slash - 1);
        this->model_pose_topic_ =
            "/model/" + model_name + "_" + std::to_string(this->gatherAgentID()) + "/odometry";
        */
}

void TacMapModule::initSubscribers() {
    if (!this->node_) {
        throw MissingExternModule();
    }

    /*
        this->sub_to_flags_topic_ = this->node_->create_subscription<px4_msgs::msg::FailsafeFlags>(
            this->flags_topic_, this->px4_qos_,
            std::bind(
                static_cast<void (TacMapModule::*)(const px4_msgs::msg::FailsafeFlags::SharedPtr)
                                const>(&TacMapModule::printData),
                this, std::placeholders::_1
            ),
            this->gatherReentrantOptions()
        );

        this->sub_to_attitude_topic_ =
        this->node_->create_subscription<px4_msgs::msg::VehicleAttitude>(
            this->attitude_topic_, this->px4_qos_,
            std::bind(
                static_cast<void (TacMapModule::*)(const px4_msgs::msg::VehicleAttitude::SharedPtr)
                                const>(&TacMapModule::printData),
                this, std::placeholders::_1
            ),
            this->gatherReentrantOptions()
        );

        this->sub_to_control_mode_topic_ = this->node_->create_subscription<
            px4_msgs::msg::VehicleControlMode>(
            this->control_mode_topic_, this->px4_qos_,
            std::bind(
                static_cast<void (TacMapModule::*)(const
                px4_msgs::msg::VehicleControlMode::SharedPtr)
                                const>(&TacMapModule::printData),
                this, std::placeholders::_1
            ),
            this->gatherReentrantOptions()
        );

        // For possible future use
        this->sub_to_model_pose_topic_ = this->node_->create_subscription<nav_msgs::msg::Odometry>(
            this->model_pose_topic_, this->standard_qos_,
            std::bind(&TacMapModule::storeInitialOffset, this, std::placeholders::_1),
            this->gatherReentrantOptions()
        );

        // For possible future use
        this->sub_to_local_pos_topic_ =
            this->node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                this->local_pos_topic_, this->px4_qos_,
                std::bind(&TacMapModule::storeLocalPosition, this, std::placeholders::_1),
                this->gatherReentrantOptions()
        );
    */

    this->sub_to_global_pos_topic_ =
        this->node_->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
            this->global_pos_topic_, this->px4_qos_,
            std::bind(&TacMapModule::storeGlobalPosition, this, std::placeholders::_1),
            this->gatherReentrantOptions()
        );

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

    this->pub_to_trajectory_setpoint_topic =
        this->node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            this->trajectory_setpoint_topic_, this->standard_qos_
        );
}

void TacMapModule::initSetup(LoggerModule* logger) {
    this->logger_ = logger;

    this->initTopics();
    this->initSubscribers();
    this->initPublishers();
}

void TacMapModule::stopService() {
    std::lock_guard<std::mutex> lock(this->running_mutex_);
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
