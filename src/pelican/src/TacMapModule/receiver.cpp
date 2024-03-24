#include "TacMapModule/tacmap.hpp"

void TacMapModule::storeGlobalPosition(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
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
    this->globalpos_buffer_.push_back(globalpos_data); // CHECK: am I using this?
}

void TacMapModule::storeOdometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    px4_msgs::msg::VehicleOdometry ned_odometry_data;
    ned_odometry_data.timestamp = msg->timestamp;
    ned_odometry_data.position = msg->position;
    ned_odometry_data.q = msg->q;
    // Other fields not needed

    std::lock_guard<std::mutex> lock(this->ned_odometry_mutex_);
    this->ned_odometry_buffer_.push_back(ned_odometry_data);
}

void TacMapModule::storeStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
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
    std::lock_guard<std::mutex> lock(this->commander_ack_mutex_);

    if (this->last_commander_ack_)
        this->last_commander_ack_.reset();
    this->last_commander_ack_ = px4_msgs::msg::VehicleCommandAck();

    this->last_commander_ack_->timestamp = msg->timestamp;
    this->last_commander_ack_->command = msg->command;
    this->last_commander_ack_->result = msg->result;
    this->last_commander_ack_->result_param1 = msg->result_param1;
    this->last_commander_ack_->result_param2 = msg->result_param2;
    this->last_commander_ack_->target_system = msg->target_system;
    this->last_commander_ack_->target_component = msg->target_component;
    this->last_commander_ack_->from_external = msg->from_external;
}

void TacMapModule::checkGlobalOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    nav_msgs::msg::Odometry enu_odometry_data;
    enu_odometry_data.header = msg->header;
    enu_odometry_data.pose = msg->pose;
    enu_odometry_data.twist = msg->twist;

    this->enu_odometry_mutex_.lock();
    this->enu_odometry_buffer_.push_back(enu_odometry_data);
    this->enu_odometry_mutex_.unlock();

    // Only compensate once each second // TODO: create a timer that does this?
    if ((unsigned int) (msg->header.stamp.sec - this->last_compensated_) >=
        constants::COMPENSATION_GAP_SECS) {
        this->sendLogDebug("Height from global odometry: {}", msg->pose.pose.position.z);
        this->last_compensated_ = msg->header.stamp.sec;
        this->signalHeightCompensation(msg->pose.pose.position.z);
    }
}
