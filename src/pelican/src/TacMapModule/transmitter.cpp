#include "TacMapModule/tacmap.hpp"

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

void TacMapModule::publishTrajectorySetpoint() {
    px4_msgs::msg::TrajectorySetpoint msg {};

    msg.position = {0.0, 0.0, -5.0};
    msg.yaw = -3.14; // [-PI:PI]

    msg.timestamp = this->gatherTime().nanoseconds() / 1000;

    this->pub_to_trajectory_setpoint_topic->publish(msg);
}

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

    int attempts = 0;

    // Continue checking for a bit, if program is not stopped
    while (this->getRunningStatus() && attempts < MAX_DATA_ATTEMPTS) {
        this->status_mutex_.lock();
        int s_length = this->status_buffer_.size();
        this->status_mutex_.unlock();

        if (s_length > 0) {
            break;
        }

        this->sendLogWarning("Status data needed not yet ready! Retrying in a bit...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        attempts++;
    };

    // Do nothing more, if stopped by CTRL-C
    if (!this->getRunningStatus())
        return;

    // If no data has been repeatedly received, stop everything
    if (attempts >= MAX_DATA_ATTEMPTS) {
        this->sendLogError("No data received from simulations! Is there one running?");
        throw std::runtime_error("No simulation detected");
    }

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
