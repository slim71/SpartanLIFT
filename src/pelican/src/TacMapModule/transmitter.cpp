/**
 * @file transmitter.cpp
 * @author Simone Vollaro (slim71sv@gmail.com)
 * @brief Methods related to data transmissions.
 * @version 1.0.0
 * @date 2024-11-14
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "TacMapModule/tacmap.hpp"

/**
 * @brief Publish a trajectory setpoint to the "/px4_{id}/fmu/in/trajectory_setpoint" topic.
 *
 * @param pos Position setpoint.
 * @param vel Velocity setpoint.
 */
void TacMapModule::publishTrajectorySetpoint(
    geometry_msgs::msg::Point pos, geometry_msgs::msg::Point vel
) {
    px4_msgs::msg::TrajectorySetpoint msg {};

    msg.position = {(float) pos.x, (float) pos.y, (float) pos.z};
    msg.velocity = {
        (float) vel.x, (float) vel.y, std::nanf("")}; // Not interested in the vertical velocity

    msg.timestamp = this->gatherTime().nanoseconds() / 1000;

    this->pub_to_trajectory_setpoint_topic_->publish(msg);
}

/**
 * @brief Publish a vehicle command to the "/px4_{id}/fmu/in/vehicle_command" topic.
 *
 * @param command
 * @param param1
 * @param param2
 * @param param3
 * @param param4
 * @param param5
 * @param param6
 * @param param7
 */
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

    this->system_id_mutex_.lock();
    msg.target_system = this->system_id_; // System which should execute the command
    this->system_id_mutex_.unlock();
    this->component_id_mutex_.lock();
    // Component which should execute the command (0 for all components)
    msg.target_component = this->component_id_;
    this->component_id_mutex_.unlock();
    // msg.source_system = sys_id;    // System sending the command
    // msg.source_component = 1; // Component sending the command

    // 0: First transmission of this command
    // 1-255: Confirmation transmissions (e.g. for kill command)
    // msg.confirmation = 0;

    msg.from_external = true; // Indicates if the command came from an external source

    msg.timestamp = this->gatherTime().nanoseconds() / 1000;

    this->pub_to_command_topic_->publish(msg);
}

/**
 * @brief Publish the message needed by PX4 to switch to offboard mode.
 *
 */
void TacMapModule::publishOffboardControlMode() {
    px4_msgs::msg::OffboardControlMode msg {};

    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;

    msg.timestamp = this->gatherTime().nanoseconds() / 1000;

    this->pub_to_offboard_control_topic_->publish(msg);
}

/**
 * @brief Wait and report whether the ack for the last command has been received from the PX4
 * commander.
 *
 * @param cmd
 * @return true
 * @return false
 */
bool TacMapModule::waitForCommanderAck(uint16_t cmd) {
    auto start_time =
        this->gatherTime().nanoseconds() / constants::NANO_TO_MILLI_CONVERSION; // [ms]
    this->sendLogDebug("ACK wait started at timestamp {}", start_time);

    // Check for a bit, in case the ACK has not been received yet
    while (true) {
        std::optional<px4_msgs::msg::VehicleCommandAck> ack = this->getCommanderAck();

        if (ack) { // at least one ACK received
            this->sendLogDebug(
                "Stored ACK found! cmd:{} timestamp:{} result: {}", ack->command, ack->timestamp,
                ack->result
            );
            if ((ack->command == cmd) &&
                (this->gatherTime().nanoseconds() / constants::NANO_TO_MILLI_CONVERSION >=
                 start_time + constants::ACK_WAIT_MILLIS)) {
                auto result =
                    ack->result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
                return result;
            } else {
                this->sendLogDebug("Unrelated/old ack found for cmd {}!", cmd);
            }
        }

        // Wait for a bit before retrying
        std::this_thread::sleep_for(std::chrono::milliseconds(constants::DELAY_MILLIS));
    }
    this->sendLogDebug("ACK wait finished");

    return false;
}
