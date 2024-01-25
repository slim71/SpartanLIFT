#include "TacMapModule/tacmap.hpp"

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

void TacMapModule::storeGlobalPosition(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
    // this->sendLogDebug(
    //     "Received GlobalPosition data! timestamp:{} lat:{} long:{} alt:{} terrain_alt:{} "
    //     "terr_valid:{}",
    //     msg->timestamp, msg->lat, msg->lon, msg->alt, msg->terrain_alt, msg->terrain_alt_valid
    // );

    px4_msgs::msg::VehicleGlobalPosition globalpos_data;
    globalpos_data.timestamp = msg->timestamp;               // [us]
    globalpos_data.timestamp_sample = msg->timestamp_sample; // [us]
    globalpos_data.lat = msg->lat;                           // [deg]
    globalpos_data.lon = msg->lon;                           // [deg]
    globalpos_data.alt = msg->alt;                           // [m]
    globalpos_data.alt_ellipsoid = msg->alt_ellipsoid;       // [m]
    globalpos_data.delta_alt = msg->delta_alt;
    globalpos_data.lat_lon_reset_counter = msg->lat_lon_reset_counter;
    globalpos_data.alt_reset_counter = msg->alt_reset_counter;
    globalpos_data.eph = msg->eph;                 // [m]
    globalpos_data.epv = msg->epv;                 // [m]
    globalpos_data.terrain_alt = msg->terrain_alt; // [m]
    globalpos_data.terrain_alt_valid = msg->terrain_alt_valid;
    globalpos_data.dead_reckoning = msg->dead_reckoning;

    std::lock_guard<std::mutex> lock(this->globalpos_mutex_);
    this->globalpos_buffer_.push_back(globalpos_data);
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

void TacMapModule::storeOdometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    // this->sendLogDebug(
    //     "Received Odometry data! timestamp:{} position:[{}, {}, {}] q:[{}, {}, {}, {}]",
    //     msg->timestamp, msg->position[0], msg->position[1], msg->position[2], msg->q[0],
    //     msg->q[1], msg->q[2], msg->q[3]
    // );Trying to send

    px4_msgs::msg::VehicleOdometry odometry_data;
    odometry_data.timestamp = msg->timestamp;
    odometry_data.position = msg->position;
    odometry_data.q = msg->q;
    // Other fields not needed

    std::lock_guard<std::mutex> lock(this->odometry_mutex_);
    this->odometry_buffer_.push_back(odometry_data);
}

void TacMapModule::storeStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    // this->sendLogDebug(
    //     "Received status data! timestamp:{} system_id:{} component_id:{}", msg->timestamp,
    //     msg->system_id, msg->component_id
    // );

    px4_msgs::msg::VehicleStatus status_data;
    status_data.timestamp = msg->timestamp;       // [us]
    status_data.armed_time = msg->armed_time;     // [us]
    status_data.takeoff_time = msg->takeoff_time; // [us]
    status_data.arming_state = msg->arming_state; // [m]
    status_data.latest_arming_reason = msg->latest_arming_reason;
    status_data.latest_disarming_reason = msg->latest_disarming_reason;
    status_data.nav_state_timestamp = msg->nav_state_timestamp; // [us]
    status_data.nav_state_user_intention = msg->nav_state_user_intention;
    status_data.nav_state = msg->nav_state;
    status_data.failure_detector_status = msg->failure_detector_status;
    status_data.hil_state = msg->hil_state;
    status_data.failsafe = msg->failsafe;
    status_data.failsafe_and_user_took_over = msg->failsafe_and_user_took_over;
    status_data.gcs_connection_lost = msg->gcs_connection_lost;
    status_data.gcs_connection_lost_counter = msg->gcs_connection_lost_counter;
    status_data.high_latency_data_link_lost = msg->high_latency_data_link_lost;
    status_data.is_vtol = msg->is_vtol;
    status_data.is_vtol_tailsitter = msg->is_vtol_tailsitter;
    status_data.in_transition_mode = msg->in_transition_mode;
    status_data.in_transition_to_fw = msg->in_transition_to_fw;
    status_data.system_type = msg->system_type;
    status_data.system_id = msg->system_id;
    status_data.component_id = msg->component_id;
    status_data.safety_button_available = msg->safety_button_available;
    status_data.safety_off = msg->safety_off;
    status_data.power_input_valid = msg->power_input_valid;
    status_data.usb_connected = msg->usb_connected;
    status_data.open_drone_id_system_present = msg->open_drone_id_system_present;
    status_data.open_drone_id_system_healthy = msg->open_drone_id_system_healthy;
    status_data.parachute_system_present = msg->parachute_system_present;
    status_data.parachute_system_healthy = msg->parachute_system_healthy;
    status_data.avoidance_system_required = msg->avoidance_system_required;
    status_data.avoidance_system_valid = msg->avoidance_system_valid;
    status_data.rc_calibration_in_progress = msg->rc_calibration_in_progress;
    status_data.calibration_enabled = msg->calibration_enabled;
    status_data.pre_flight_checks_pass = msg->pre_flight_checks_pass;

    std::lock_guard<std::mutex> status_lock(this->status_mutex_);
    std::lock_guard<std::mutex> sysid_lock(this->system_id_mutex_);
    std::lock_guard<std::mutex> compid_lock(this->component_id_mutex_);
    this->status_buffer_.push_back(status_data);
    // While we're at it, let's store the IDs identifying this agent
    this->system_id_ = msg->system_id;
    this->component_id_ = msg->component_id;
}

void TacMapModule::storeAck(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg) {
    this->sendLogDebug(
        "Received ACK! command:{} result:{} param1:{} param2:{} system:{} component:{}",
        msg->command, msg->result, msg->result_param1, msg->result_param2, msg->target_system,
        msg->target_component
    );

    std::lock_guard<std::mutex> lock(this->ack_mutex_);

    if (this->last_ack_)
        this->last_ack_.reset();
    this->last_ack_ = px4_msgs::msg::VehicleCommandAck();

    this->last_ack_->timestamp = msg->timestamp;
    this->last_ack_->command = msg->command;
    this->last_ack_->result = msg->result;
    this->last_ack_->result_param1 = msg->result_param1;
    this->last_ack_->result_param2 = msg->result_param2;
    this->last_ack_->target_system = msg->target_system;
    this->last_ack_->target_component = msg->target_component;
    this->last_ack_->from_external = msg->from_external;
}
