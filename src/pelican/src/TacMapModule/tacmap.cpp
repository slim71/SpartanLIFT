#include "TacMapModule/tacmap.hpp"
#include "PelicanModule/pelican.hpp"

/************************** Ctors/Dctors ***************************/
TacMapModule::TacMapModule() {
    this->node_ = nullptr;
}

// TODO_ add logger to init values (as in ElectionModule)
TacMapModule::TacMapModule(Pelican* node) : node_(node) {}

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
    this->status_topic_ = px4_header + "/fmu/out/vehicle_status"s;
    this->command_topic_ = px4_header + "/fmu/in/vehicle_command"s;

    this->offboard_control_topic_ = px4_header + "/fmu/in/offboard_control_mode"s;
    this->trajectory_setpoint_topic_ = px4_header + "/fmu/in/trajectory_setpoint"s;
}

void TacMapModule::initSubscribers() {
    // TODO: change handle functions

    this->sub_to_flags_topic_ = this->node_->create_subscription<px4_msgs::msg::FailsafeFlags>(
        this->flags_topic_, this->px4_qos_,
        std::bind(
            static_cast<void (TacMapModule::*)(const px4_msgs::msg::FailsafeFlags::SharedPtr)
                            const>(&TacMapModule::printData),
            this, std::placeholders::_1
        ),
        this->gatherReentrantOptions()
    );

    this->sub_to_attitude_topic_ = this->node_->create_subscription<px4_msgs::msg::VehicleAttitude>(
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
            static_cast<void (TacMapModule::*)(const px4_msgs::msg::VehicleControlMode::SharedPtr)
                            const>(&TacMapModule::printData),
            this, std::placeholders::_1
        ),
        this->gatherReentrantOptions()
    );

    this->sub_to_global_pos_topic_ = this->node_->create_subscription<
        px4_msgs::msg::VehicleGlobalPosition>(
        this->global_pos_topic_, this->px4_qos_,
        std::bind(
            static_cast<void (TacMapModule::*)(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr
            ) const>(&TacMapModule::printData),
            this, std::placeholders::_1
        ),
        this->gatherReentrantOptions()
    );

    this->sub_to_gps_pos_topic_ = this->node_->create_subscription<px4_msgs::msg::SensorGps>(
        this->gps_pos_topic_, this->px4_qos_,
        std::bind(
            static_cast<void (TacMapModule::*)(const px4_msgs::msg::SensorGps::SharedPtr) const>(
                &TacMapModule::printData
            ),
            this, std::placeholders::_1
        ),
        this->gatherReentrantOptions()
    );

    // In NED. The coordinate system origin is the vehicle position at the time when the EKF2-module
    // was started. Needed by every type of agent and never canceled
    this->sub_to_local_pos_topic_ = this->node_->create_subscription<
        px4_msgs::msg::VehicleLocalPosition>(
        this->local_pos_topic_, this->px4_qos_,
        std::bind(
            static_cast<void (TacMapModule::*)(const px4_msgs::msg::VehicleLocalPosition::SharedPtr)
                            const>(&TacMapModule::printData),
            this, std::placeholders::_1
        ),
        this->gatherReentrantOptions()
    );

    this->sub_to_odometry_topic_ = this->node_->create_subscription<px4_msgs::msg::VehicleOdometry>(
        this->odometry_topic_, this->px4_qos_,
        std::bind(
            static_cast<void (TacMapModule::*)(const px4_msgs::msg::VehicleOdometry::SharedPtr)
                            const>(&TacMapModule::printData),
            this, std::placeholders::_1
        ),
        this->gatherReentrantOptions()
    );

    this->sub_to_status_topic_ = this->node_->create_subscription<px4_msgs::msg::VehicleStatus>(
        this->status_topic_, this->px4_qos_,
        std::bind(
            static_cast<void (TacMapModule::*)(const px4_msgs::msg::VehicleStatus::SharedPtr)
                            const>(&TacMapModule::printData),
            this, std::placeholders::_1
        ),
        this->gatherReentrantOptions()
    );
}

void TacMapModule::initPublishers() {
    this->pub_to_command_topic_ = this->node_->create_publisher<px4_msgs::msg::VehicleCommand>(
        this->command_topic_, 10
    ); // TODO: put 10 in a standard_qos variable

    this->pub_to_offboard_control_topic_ =
        this->node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
            this->offboard_control_topic_, 10
        ); // TODO: put 10 in a standard_qos variable

    this->pub_to_trajectory_setpoint_topic =
        this->node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            this->trajectory_setpoint_topic_, 10
        ); // TODO: put 10 in a standard_qos variable

    // TODO: test and delete from here
    this->publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF);
}

void TacMapModule::initSetup(LoggerModule* logger) {
    /* ... */

    this->logger_ = logger;

    // TODO: setup all topics, subscribers and publishers here
    this->initTopics();
    this->initSubscribers();
    this->initPublishers();

    // TODO: put constant in header
    this->offboard_timer_ = this->node_->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&TacMapModule::offboardTimerCallback, this)
    );
}

/****************************** Other ******************************/
void TacMapModule::printData(const px4_msgs::msg::FailsafeFlags::SharedPtr msg) const {
    std::cout << "\n\n";
    std::cout << "RECEIVED FailsafeFlags DATA" << std::endl;
    std::cout << "=============================" << std::endl;
    std::cout << "Time since start: " << msg->timestamp << std::endl; // [us]
    std::cout << "mode_req_angular_velocity: " << msg->mode_req_angular_velocity << std::endl;
    std::cout << "mode_req_attitude: " << msg->mode_req_attitude << std::endl;
    std::cout << "mode_req_local_alt: " << msg->mode_req_local_alt << std::endl;
    std::cout << "mode_req_local_position: " << msg->mode_req_local_position << std::endl;
    std::cout << "mode_req_local_position_relaxed: " << msg->mode_req_local_position_relaxed
              << std::endl;
    std::cout << "mode_req_global_position: " << msg->mode_req_global_position << std::endl;
    std::cout << "mode_req_mission: " << msg->mode_req_mission << std::endl;
    std::cout << "mode_req_offboard_signal: " << msg->mode_req_offboard_signal << std::endl;
    std::cout << "mode_req_home_position: " << msg->mode_req_home_position << std::endl;
    std::cout << "mode_req_wind_and_flight_time_compliance: "
              << msg->mode_req_wind_and_flight_time_compliance << std::endl;
    std::cout << "mode_req_prevent_arming: " << msg->mode_req_prevent_arming << std::endl;
    std::cout << "mode_req_manual_control: " << msg->mode_req_manual_control << std::endl;
    std::cout << "mode_req_other: " << msg->mode_req_other << std::endl;
    std::cout << "angular_velocity_invalid: " << msg->angular_velocity_invalid << std::endl;
    std::cout << "attitude_invalid: " << msg->attitude_invalid << std::endl;
    std::cout << "local_altitude_invalid: " << msg->local_altitude_invalid << std::endl;
    std::cout << "local_position_invalid: " << msg->local_position_invalid << std::endl;
    std::cout << "local_position_invalid_relaxed: " << msg->local_position_invalid_relaxed
              << std::endl;
    std::cout << "local_velocity_invalid: " << msg->local_velocity_invalid << std::endl;
    std::cout << "global_position_invalid: " << msg->global_position_invalid << std::endl;
    std::cout << "auto_mission_missing: " << msg->auto_mission_missing << std::endl;
    std::cout << "offboard_control_signal_lost: " << msg->offboard_control_signal_lost << std::endl;
    std::cout << "home_position_invalid: " << msg->home_position_invalid << std::endl;
    std::cout << "manual_control_signal_lost: " << msg->manual_control_signal_lost << std::endl;
    std::cout << "gcs_connection_lost: " << msg->gcs_connection_lost << std::endl;
    std::cout << "battery_warning: " << msg->battery_warning << std::endl;
    std::cout << "battery_low_remaining_time: " << msg->battery_low_remaining_time << std::endl;
    std::cout << "battery_unhealthy: " << msg->battery_unhealthy << std::endl;
    std::cout << "primary_geofence_breached: " << msg->primary_geofence_breached << std::endl;
    std::cout << "mission_failure: " << msg->mission_failure << std::endl;
    std::cout << "vtol_fixed_wing_system_failure: " << msg->vtol_fixed_wing_system_failure
              << std::endl;
    std::cout << "wind_limit_exceeded: " << msg->wind_limit_exceeded << std::endl;
    std::cout << "flight_time_limit_exceeded: " << msg->flight_time_limit_exceeded << std::endl;
    std::cout << "local_position_accuracy_low: " << msg->local_position_accuracy_low << std::endl;
    std::cout << "fd_critical_failure: " << msg->fd_critical_failure << std::endl;
    std::cout << "fd_esc_arming_failure: " << msg->fd_esc_arming_failure << std::endl;
    std::cout << "fd_imbalanced_prop: " << msg->fd_imbalanced_prop << std::endl;
    std::cout << "fd_motor_failure: " << msg->fd_motor_failure << std::endl;
}

void TacMapModule::printData(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) const {
    std::cout << "\n\n";
    std::cout << "RECEIVED VehicleAttitude DATA" << std::endl;
    std::cout << "=============================" << std::endl;
    std::cout << "Time since start: " << msg->timestamp << std::endl;             // [us]
    std::cout << "Timestamp of raw data: " << msg->timestamp_sample << std::endl; // [us]
    std::cout << "q: " << msg->q[0] << " " << msg->q[1] << " " << msg->q[2] << " " << msg->q[3]
              << std::endl;
    std::cout << "delta_q_reset: " << msg->delta_q_reset[0] << " " << msg->delta_q_reset[1] << " "
              << msg->delta_q_reset[2] << " " << msg->delta_q_reset[3] << std::endl;
    std::cout << "quat_reset_counter: " << msg->quat_reset_counter << std::endl;
}

void TacMapModule::printData(const px4_msgs::msg::VehicleControlMode::SharedPtr msg) const {
    std::cout << "\n\n";
    std::cout << "RECEIVED VehicleControlMode DATA" << std::endl;
    std::cout << "=============================" << std::endl;
    std::cout << "Time since start: " << msg->timestamp << std::endl; // [us]
    std::cout << "flag_armed: " << msg->flag_armed << std::endl;
    std::cout << "flag_multicopter_position_control_enabled: "
              << msg->flag_multicopter_position_control_enabled << std::endl;
    std::cout << "flag_control_manual_enabled: " << msg->flag_control_manual_enabled << std::endl;
    std::cout << "flag_control_auto_enabled: " << msg->flag_control_auto_enabled << std::endl;
    std::cout << "flag_control_offboard_enabled: " << msg->flag_control_offboard_enabled
              << std::endl;
    std::cout << "flag_control_rates_enabled: " << msg->flag_control_rates_enabled << std::endl;
    std::cout << "flag_control_attitude_enabled: " << msg->flag_control_attitude_enabled
              << std::endl;
    std::cout << "flag_control_acceleration_enabled: " << msg->flag_control_acceleration_enabled
              << std::endl;
    std::cout << "flag_control_velocity_enabled: " << msg->flag_control_velocity_enabled
              << std::endl;
    std::cout << "flag_control_position_enabled: " << msg->flag_control_position_enabled
              << std::endl;
    std::cout << "flag_control_altitude_enabled: " << msg->flag_control_altitude_enabled
              << std::endl;
    std::cout << "flag_control_climb_rate_enabled: " << msg->flag_control_climb_rate_enabled
              << std::endl;
    std::cout << "flag_control_termination_enabled: " << msg->flag_control_termination_enabled
              << std::endl;
}

void TacMapModule::printData(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) const {
    std::cout << "\n\n";
    std::cout << "RECEIVED VehicleGlobalPosition DATA" << std::endl;
    std::cout << "=============================" << std::endl;
    std::cout << "Time since start: " << msg->timestamp << std::endl;             // [us]
    std::cout << "Timestamp of raw data: " << msg->timestamp_sample << std::endl; // [us]
    std::cout << "lat: " << msg->lat << std::endl;                                // [deg]
    std::cout << "lon: " << msg->lon << std::endl;                                // [deg]
    std::cout << "alt: " << msg->alt << std::endl;                                // [m]
    std::cout << "alt_ellipsoid: " << msg->alt_ellipsoid << std::endl;
    std::cout << "delta_alt: " << msg->delta_alt << std::endl;
    std::cout << "lat_lon_reset_counter: " << msg->lat_lon_reset_counter << std::endl;
    std::cout << "alt_reset_counter: " << msg->alt_reset_counter << std::endl;
    std::cout << "eph: " << msg->eph << std::endl;                 // [m]
    std::cout << "epv: " << msg->epv << std::endl;                 // [m]
    std::cout << "terrain_alt: " << msg->terrain_alt << std::endl; // [m]
    std::cout << "terrain_alt_valid: " << msg->terrain_alt_valid << std::endl;
    std::cout << "dead_reckoning: " << msg->dead_reckoning << std::endl;
}

void TacMapModule::printData(const px4_msgs::msg::SensorGps::SharedPtr msg) const {
    std::cout << "\n\n";
    std::cout << "RECEIVED SensorGps DATA" << std::endl;
    std::cout << "=============================" << std::endl;
    std::cout << "Time since start: " << msg->timestamp << std::endl;             // [us]
    std::cout << "Timestamp of raw data: " << msg->timestamp_sample << std::endl; // [us]
    std::cout << "device_id: " << msg->device_id << std::endl;
    std::cout << "latitude_deg: " << msg->lat << std::endl;                       // [deg]
    std::cout << "longitude_deg: " << msg->lon << std::endl;                      // [deg]
    std::cout << "altitude_msl_m: " << msg->alt << std::endl;                     // [m]
    std::cout << "altitude_ellipsoid_m: " << msg->alt_ellipsoid << std::endl;     // [m]
    std::cout << "s_variance_m_s: " << msg->s_variance_m_s << std::endl;          // [m/s]
    std::cout << "c_variance_rad: " << msg->c_variance_rad << std::endl;          // [rad]
    std::cout << "fix_type: " << msg->fix_type << std::endl;
    std::cout << "eph: " << msg->eph << std::endl;                                // [m]
    std::cout << "epv: " << msg->epv << std::endl;                                // [m]
    std::cout << "hdop: " << msg->hdop << std::endl;
    std::cout << "vdop: " << msg->vdop << std::endl;
    std::cout << "noise_per_ms: " << msg->noise_per_ms << std::endl;
    std::cout << "automatic_gain_control: " << msg->automatic_gain_control << std::endl;
    std::cout << "jamming_state: " << msg->jamming_state << std::endl;
    std::cout << "jamming_indicator: " << msg->jamming_indicator << std::endl;
    std::cout << "spoofing_state: " << msg->spoofing_state << std::endl;
    std::cout << "vel_m_s: " << msg->vel_m_s << std::endl;                                 // [m/s]
    std::cout << "vel_n_m_s: " << msg->vel_n_m_s << std::endl;                             // [m/s]
    std::cout << "vel_e_m_s: " << msg->vel_e_m_s << std::endl;                             // [m/s]
    std::cout << "vel_d_m_s: " << msg->vel_d_m_s << std::endl;                             // [m/s]
    std::cout << "cog_rad: " << msg->cog_rad << std::endl;                                 // [rad]
    std::cout << "vel_ned_valid: " << msg->vel_ned_valid << std::endl;
    std::cout << "timestamp_time_relative: " << msg->timestamp_time_relative << std::endl; // [us]
    std::cout << "time_utc_usec: " << msg->time_utc_usec << std::endl;                     // [us]
    std::cout << "satellites_used: " << msg->satellites_used << std::endl;
    std::cout << "heading: " << msg->heading << std::endl;                                 // [rad]
    std::cout << "heading_offset: " << msg->heading_offset << std::endl;                   // [rad]
    std::cout << "heading_accuracy: " << msg->heading_accuracy << std::endl;               // [rad]
    std::cout << "rtcm_injection_rate: " << msg->rtcm_injection_rate << std::endl;
    std::cout << "selected_rtcm_instance: " << msg->selected_rtcm_instance << std::endl;
    std::cout << "rtcm_crc_failed: " << msg->rtcm_crc_failed << std::endl;
    std::cout << "rtcm_msg_used: " << msg->rtcm_msg_used << std::endl;
}

void TacMapModule::printData(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) const {
    std::cout << "\n\n";
    std::cout << "RECEIVED VehicleLocalPosition DATA" << std::endl;
    std::cout << "=============================" << std::endl;
    std::cout << "Time since start: " << msg->timestamp << std::endl;             // [us]
    std::cout << "Timestamp of raw data: " << msg->timestamp_sample << std::endl; // [us]
    std::cout << "x: " << msg->x << std::endl;                                    // [m]
    std::cout << "y: " << msg->y << std::endl;                                    // [m]
    std::cout << "z: " << msg->z << std::endl;                                    // [m]
    std::cout << "vx: " << msg->vx << std::endl;                                  // [m/s]
    std::cout << "vy: " << msg->vy << std::endl;                                  // [m/s]
    std::cout << "vz: " << msg->vz << std::endl;                                  // [m/s]
    std::cout << "xy_valid: " << msg->xy_valid << std::endl;   // true if x and y are valid
    std::cout << "z_valid: " << msg->z_valid << std::endl;     // true if z is valid
    std::cout << "v_xy_valid: " << msg->xy_valid << std::endl; // true if vx and vy are valid
    std::cout << "v_z_valid: " << msg->z_valid << std::endl;   // true if vz is valid
    std::cout << "ax: " << msg->ax << std::endl;               // [m/s^2]
    std::cout << "ay: " << msg->ay << std::endl;               // [m/s^2]
    std::cout << "az: " << msg->az << std::endl;               // [m/s^2]
    std::cout << "heading: " << msg->heading << std::endl;     // [rad]
}

void TacMapModule::printData(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) const {
    std::cout << "\n\n";
    std::cout << "RECEIVED VehicleOdometry DATA" << std::endl;
    std::cout << "=============================" << std::endl;
    std::cout << "Time since start: " << msg->timestamp << std::endl;             // [us]
    std::cout << "Timestamp of raw data: " << msg->timestamp_sample << std::endl; // [us]
    std::cout << "pose_frame: " << msg->pose_frame << std::endl;                  // [m]
    std::cout << "position: " << msg->position[0] << " " << msg->position[1] << " "
              << msg->position[2] << std::endl;                                   // [m]
    std::cout << "q: " << msg->q[0] << " " << msg->q[1] << " " << msg->q[2] << " " << msg->q[3]
              << " " << std::endl;
    std::cout << "velocity_frame: " << msg->velocity_frame << std::endl; // [m/s]
    std::cout << "velocity: " << msg->velocity[0] << " " << msg->velocity[1] << " "
              << msg->velocity[2] << " " << std::endl;                   // [m/s]
    std::cout << "angular_velocity: " << msg->angular_velocity[0] << " " << msg->angular_velocity[1]
              << " " << msg->angular_velocity[2] << " " << std::endl;    // [rad/s]
    std::cout << "position_variance: " << msg->position_variance[0] << " "
              << msg->position_variance[1] << " " << msg->position_variance[2] << " " << std::endl;
    std::cout << "orientation_variance: " << msg->orientation_variance[0] << " "
              << msg->orientation_variance[1] << " " << msg->orientation_variance[2] << " "
              << std::endl;
    std::cout << "velocity_variance: " << msg->velocity_variance[0] << " "
              << msg->velocity_variance[1] << " " << msg->velocity_variance[2] << " " << std::endl;
    std::cout << "reset_counter: " << msg->reset_counter << std::endl;
    std::cout << "quality: " << msg->quality << std::endl;
}

void TacMapModule::printData(const px4_msgs::msg::VehicleStatus::SharedPtr msg) const {
    std::cout << "\n\n";
    std::cout << "RECEIVED VehicleStatus DATA" << std::endl;
    std::cout << "=============================" << std::endl;
    std::cout << "Time since start: " << msg->timestamp << std::endl; // [us]
    std::cout << "armed_time: " << msg->armed_time << std::endl;      // [us]
    std::cout << "takeoff_time: " << msg->takeoff_time << std::endl;  // [us]
    std::cout << "arming_state: " << msg->arming_state << std::endl;  // [m]
    std::cout << "latest_arming_reason: " << msg->latest_arming_reason << std::endl;
    std::cout << "latest_disarming_reason: " << msg->latest_disarming_reason << std::endl;
    std::cout << "nav_state_timestamp: " << msg->nav_state_timestamp << std::endl; // [us]
    std::cout << "nav_state_user_intention: " << msg->nav_state_user_intention << std::endl;
    std::cout << "nav_state: " << msg->nav_state << std::endl;
    std::cout << "failure_detector_status: " << msg->failure_detector_status << std::endl;
    std::cout << "hil_state: " << msg->hil_state << std::endl;
    std::cout << "failsafe: " << msg->failsafe << std::endl;
    std::cout << "failsafe_and_user_took_over: " << msg->failsafe_and_user_took_over << std::endl;
    std::cout << "gcs_connection_lost: " << msg->gcs_connection_lost << std::endl;
    std::cout << "gcs_connection_lost_counter: " << msg->gcs_connection_lost_counter << std::endl;
    std::cout << "high_latency_data_link_lost: " << msg->high_latency_data_link_lost << std::endl;
    std::cout << "is_vtol: " << msg->is_vtol << std::endl;
    std::cout << "is_vtol_tailsitter: " << msg->is_vtol_tailsitter << std::endl;
    std::cout << "in_transition_mode: " << msg->in_transition_mode << std::endl;
    std::cout << "in_transition_to_fw: " << msg->in_transition_to_fw << std::endl;
    std::cout << "system_type: " << msg->system_type << std::endl;
    std::cout << "system_id: " << msg->system_id << std::endl;
    std::cout << "component_id: " << msg->component_id << std::endl;
    std::cout << "safety_button_available: " << msg->safety_button_available << std::endl;
    std::cout << "safety_off: " << msg->safety_off << std::endl;
    std::cout << "power_input_valid: " << msg->power_input_valid << std::endl;
    std::cout << "usb_connected: " << msg->usb_connected << std::endl;
    std::cout << "open_drone_id_system_present: " << msg->open_drone_id_system_present << std::endl;
    std::cout << "open_drone_id_system_healthy: " << msg->open_drone_id_system_healthy << std::endl;
    std::cout << "parachute_system_present: " << msg->parachute_system_present << std::endl;
    std::cout << "parachute_system_healthy: " << msg->parachute_system_healthy << std::endl;
    std::cout << "avoidance_system_required: " << msg->avoidance_system_required << std::endl;
    std::cout << "avoidance_system_valid: " << msg->avoidance_system_valid << std::endl;
    std::cout << "rc_calibration_in_progress: " << msg->rc_calibration_in_progress << std::endl;
    std::cout << "calibration_enabled: " << msg->calibration_enabled << std::endl;
    std::cout << "pre_flight_checks_pass: " << msg->pre_flight_checks_pass << std::endl;
}

void TacMapModule::publishVehicleCommand(uint16_t command, float param1, float param2) {
    px4_msgs::msg::VehicleCommand msg {};

    msg.param1 = param1;
    msg.param2 = param2;
    // Other parameters are not relevant for vehicle commands

    msg.command = command; // Command ID

    // PX4 accepts VehicleCommand messages only if their target_system field is zero (broadcast
    // communication) or coincides with MAV_SYS_ID. In all other situations, the messages are
    // ignored. For example, if you want to send a command to your third vehicle, which has
    // px4_instance=2, you need to set target_system=3 in all your VehicleCommand messages.
    msg.target_system = this->gatherAgentID(); // System which should execute the command
    msg.target_component = 1; // Component which should execute the command, 0 for all components
    msg.source_system = 1;    // System sending the command
    msg.source_component = 1; // Component sending the command

    // msg.confirmation = 0; // 0: First transmission of this command. 1-255: Confirmation
    // transmissions (e.g. for kill command)

    msg.from_external = true; // Indicates if the command came from an external source

    msg.timestamp = this->gatherTime().nanoseconds() / 1000;

    this->pub_to_command_topic_->publish(msg);
}

void TacMapModule::arm() {
    // TODO: use defines
    this->publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0
    );

    this->sendLogInfo("Arm command send");
}

void TacMapModule::disarm() {
    // TODO: use defines
    this->publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0
    );

    this->sendLogInfo("Disarm command send");
}

void TacMapModule::publishOffboardControlMode() {
    // PX4 requires that the vehicle is already receiving OffboardControlMode messages
    // before it will arm in offboard mode, or before it will switch to offboard mode
    // when flying. In addition, PX4 will switch out of offboard mode if the stream rate
    // of OffboardControlMode messages drops below approximately 2Hz.

    px4_msgs::msg::OffboardControlMode msg {};

    // The OffboardControlMode is required in order to inform PX4 of the type of offboard
    // control behing used. Here we're only using position control, so the position field
    // is set to true and all the other fields are set to false.
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;

    msg.timestamp = this->gatherTime().nanoseconds() / 1000;

    this->pub_to_offboard_control_topic_->publish(msg);
}

void TacMapModule::offboardTimerCallback() {
    if (this->offboard_setpoint_counter_ == 10) {
        // Change to Offboard mode after 10 setpoints
        this->publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

        // Arm the vehicle
        this->arm();
    }

    // offboard_control_mode needs to be paired with trajectory_setpoint
    this->publishOffboardControlMode();
    this->publishTrajectorySetpoint();

    // stop the counter after reaching 11
    if (this->offboard_setpoint_counter_ < 11) {
        this->offboard_setpoint_counter_++;
    }
}

void TacMapModule::publishTrajectorySetpoint() {
    px4_msgs::msg::TrajectorySetpoint msg {};

    msg.position = {0.0, 0.0, -5.0};
    msg.yaw = -3.14; // [-PI:PI]

    msg.timestamp = this->gatherTime().nanoseconds() / 1000;

    this->pub_to_trajectory_setpoint_topic->publish(msg);
}
