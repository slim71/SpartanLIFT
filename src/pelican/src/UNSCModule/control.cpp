#include "UNSCModule/unsc.hpp"
#include <math.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/transform_datatypes.h>

void UNSCModule::arm() {
    this->signalPublishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        px4_msgs::msg::VehicleCommand::ARMING_ACTION_ARM
    );

    this->sendLogInfo("Arm command sent");
}

void UNSCModule::disarm() {
    this->signalPublishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        px4_msgs::msg::VehicleCommand::ARMING_ACTION_DISARM
    );

    this->sendLogInfo("Disarm command sent");
}

// void UNSCModule::offboardTimerCallback() {
//     if (this->offboard_setpoint_counter_ == 10) {
//         // Change to Offboard mode after 10 setpoints
//         this->sendLogDebug("Changing to Offboard mode!");
//         this->publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1,
//         6);

// // Arm the vehicle
// this->arm();
// }

// // offboard_control_mode needs to be paired with trajectory_setpoint
// this->publishOffboardControlMode();
// this->publishTrajectorySetpoint();

// // stop the counter after reaching 11
// if (this->offboard_setpoint_counter_ < 11) {
//     this->offboard_setpoint_counter_++;
// }
// }

void UNSCModule::takeoff() {
    int attempts = 0;

    std::optional<px4_msgs::msg::VehicleGlobalPosition> pos;
    std::optional<px4_msgs::msg::VehicleOdometry> odo;
    // Continue checking for a bit, if program is not stopped
    while (this->checkIsRunning() && attempts < MAX_DATA_ATTEMPTS) {
        pos = this->gatherGlobalPosition();
        odo = this->gatherOdometry();

        if (pos && odo) {
            break;
        }

        this->sendLogWarning(
            "Data needed not yet ready (globalpos:{} odometry:{})! Retrying in a bit...",
            pos ? 1 : 0, odo ? 1 : 0
        );
        std::this_thread::sleep_for(std::chrono::seconds(1));
        attempts++;
    };

    // Do nothing more, if stopped by CTRL-C
    if (!this->checkIsRunning())
        return;

    // If no data has been repeatedly received, stop everything
    if (attempts >= MAX_DATA_ATTEMPTS) {
        this->sendLogError("No data received from simulations! Is there one running?");
        throw std::runtime_error("No simulation detected");
    }

    auto lat = pos->lat;
    auto lon = pos->lon;
    auto alt = pos->alt;

    // Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
    auto quat = odo->q;
    tf2::Quaternion tf2quat(quat[1], quat[2], quat[3], quat[0]);
    tf2::Matrix3x3 mat(tf2quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    auto yaw_deg = yaw * 180.0 / M_PI;

    this->signalPublishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 0.0, 0.0, 0.0, yaw_deg, lat, lon,
        alt + 1
    );

    if (waitForAck(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF)) {
        this->arm();
    }
}

bool UNSCModule::waitForAck(uint16_t cmd) {
    auto start_time = this->gatherTime().nanoseconds() / 1000000; // [ms]
    this->sendLogDebug("ACK wait started at timestamp {}", start_time);

    // Check for a bit, in case the ACK has not been received yet
    while (this->gatherTime().nanoseconds() / 1000000 - start_time < 100) {
        std::optional<px4_msgs::msg::VehicleCommandAck> ack = this->gatherAck();

        if (ack) { // at least one ACK received
            this->sendLogDebug(
                "ACK stored found! cmd:{} timestamp:{}", ack->command, ack->timestamp
            );
            if ((ack->command == cmd)) {
                auto result =
                    ack->result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
                return result;
            }
        }

        // Wait for a bit before retrying
        std::this_thread::sleep_for(std::chrono::microseconds(10000));
    }
    this->sendLogDebug("ACK wait finished");

    return false;
}
