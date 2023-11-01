
#include "TacMapModule/tacmap.hpp"
#include <math.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/transform_datatypes.h>

void TacMapModule::publishVehicleCommand(
    uint16_t command, float param1, float param2, float param3, float param4, float param5,
    float param6, float param7
) {
    px4_msgs::msg::VehicleCommand msg {};

    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
    msg.param7 = param7;

    msg.command = command; // Command ID

    while (true) {
        this->status_mutex_.lock();
        int s_length = this->status_buffer_.size();
        this->status_mutex_.unlock();

        if (s_length > 0) {
            break;
        }

        this->sendLogWarning("Data needed not yet ready! Retrying in a bit...");
        std::this_thread::sleep_for(std::chrono::seconds(1)); // CHECK: less time?
    };

    this->status_mutex_.lock();
    auto last_status = this->status_buffer_.back();
    this->status_mutex_.unlock();

    msg.target_system = last_status.system_id; // System which should execute the command
    msg.target_component =
        last_status
            .component_id; // Component which should execute the command, 0 for all components
    // msg.source_system = this->gatherAgentID();    // System sending the command
    // msg.source_component = 1; // Component sending the command

    // msg.confirmation = 0; // 0: First transmission of this command. 1-255: Confirmation
    // transmissions (e.g. for kill command)

    msg.from_external = true; // Indicates if the command came from an external source

    msg.timestamp = this->gatherTime().nanoseconds() / 1000;

    this->pub_to_command_topic_->publish(msg);
}

void TacMapModule::arm() {
    this->publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        px4_msgs::msg::VehicleCommand::ARMING_ACTION_ARM
    );

    this->sendLogInfo("Arm command sent");
}

void TacMapModule::disarm() {
    this->publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        px4_msgs::msg::VehicleCommand::ARMING_ACTION_DISARM
    );

    this->sendLogInfo("Disarm command sent");
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
        this->sendLogDebug("Changing to Offboard mode!");
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

void TacMapModule::takeoff() {
    this->offboard_timer_.reset();

    // TODO: stop after tot attempts and/or at CTL-C
    while (true) {
        this->globalpos_mutex_.lock();
        int g_length = this->globalpos_buffer_.size();
        this->globalpos_mutex_.unlock();
        this->odometry_mutex_.lock();
        int o_lenght = this->odometry_buffer_.size();
        this->odometry_mutex_.unlock();

        if ((g_length > 0) && (o_lenght > 0)) {
            break;
        }

        this->sendLogWarning("Data needed not yet ready! Retrying in a bit...");
        std::this_thread::sleep_for(std::chrono::seconds(1)); // CHECK: less time?
    };

    this->globalpos_mutex_.lock();
    auto lat = this->globalpos_buffer_.back().lat;
    auto lon = this->globalpos_buffer_.back().lon;
    auto alt = this->globalpos_buffer_.back().alt;
    this->globalpos_mutex_.unlock();

    this->odometry_mutex_.lock();
    // Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
    auto quat = this->odometry_buffer_.back().q;
    tf2::Quaternion tf2quat(quat[1], quat[2], quat[3], quat[0]);
    this->odometry_mutex_.unlock();
    tf2::Matrix3x3 mat(tf2quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    auto yaw_deg = yaw * 180.0 / M_PI;

    this->publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 0.0, 0.0, 0.0, yaw_deg, lat, lon,
        alt + 1
    );

    if (waitForAck(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF)) {
        this->arm();
    }
}

bool TacMapModule::waitForAck(uint16_t cmd) {
    auto start_time = this->gatherTime().nanoseconds() / 1000000; // [ms]
    this->sendLogInfo("ACK wait started at timestamp {}", start_time);

    // Check for a bit, in case the ACK has not been received yet
    while (this->gatherTime().nanoseconds() / 1000000 - start_time < 100) {
        this->ack_mutex_.lock();

        if (this->last_ack_) { // at least one ACK received
            this->sendLogInfo(
                "ACK stored found! cmd:{} timestamp:{}", this->last_ack_->command,
                this->last_ack_->timestamp
            );
            if ((this->last_ack_->command == cmd)) {
                auto result = this->last_ack_->result ==
                              px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
                this->ack_mutex_.unlock();
                return result;
            }
        }

        this->ack_mutex_.unlock();

        // Wait for a bit before retrying
        std::this_thread::sleep_for(std::chrono::microseconds(10000));
    }
    this->sendLogInfo("ACK wait finished");

    return false;
}
