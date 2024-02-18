#include "PelicanModule/pelican.hpp"
#include "UNSCModule/unsc.hpp"

bool UNSCModule::arm() {
    return this->sendToCommanderUnit(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        px4_msgs::msg::VehicleCommand::ARMING_ACTION_ARM
    );
}

bool UNSCModule::disarm() {
    cancelTimer(this->offboard_timer_);
    return this->sendToCommanderUnit(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        px4_msgs::msg::VehicleCommand::ARMING_ACTION_DISARM
    );
}

// Pitch| Empty| Empty| Yaw| Latitude| Longitude| Altitude|
bool UNSCModule::takeoff(unsigned int height) {
    cancelTimer(this->offboard_timer_);
    return this->sendToCommanderUnit(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, NAN, NAN, NAN, NAN, NAN, NAN,
        (height > 0) ? height : NAN
    );
}

// Empty| Empty| Empty| Yaw| Latitude| Longitude| Altitude|
bool UNSCModule::land() {
    cancelTimer(this->offboard_timer_);
    return this->sendToCommanderUnit(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
}

// Use current?| Empty| Empty| Empty| Latitude| Longitude| Altitude|
bool UNSCModule::setHome() {
    return this->sendToCommanderUnit(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_HOME, constants::CONFIRM_SET_HOME
    );
}

// Empty| Empty| Empty| Empty| Empty| Empty| Empty|
bool UNSCModule::returnToLaunchPosition() {
    cancelTimer(this->offboard_timer_);
    return this->sendToCommanderUnit(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
    );
}

void UNSCModule::runPreChecks() {
    // Delete wall timer to have it set off only once
    cancelTimer(this->starting_timer_);

    this->signalPublishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_RUN_PREARM_CHECKS);

    auto start_time =
        this->gatherTime().nanoseconds() / constants::NANO_TO_MILLI_ORDER_CONVERSION; // [ms]
    this->sendLogDebug("Status data wait started at timestamp {}", start_time);

    // Check for a bit, in case the ACK has not been received yet
    while (this->gatherTime().nanoseconds() / constants::NANO_TO_MILLI_ORDER_CONVERSION -
               start_time <
           constants::ACK_WAIT_MILLIS) {
        std::optional<px4_msgs::msg::VehicleStatus> status = this->gatherStatus();

        if (status) { // At least one status received
            this->sendLogDebug(
                "Stored status message found! timestamp:{} arming_state:{} nav_state:{} "
                "pre_flight_checks_pass:{} failsafe:{}",
                status->timestamp, status->arming_state, status->nav_state,
                status->pre_flight_checks_pass, status->failsafe
            );

            // Check whether the agent in the simulation is good to go
            this->sitl_ready_ =
                status->pre_flight_checks_pass && !status->failsafe &&
                ((1u << status->nav_state) != 0) && // Taken from the PX4 commander
                status->arming_state < px4_msgs::msg::VehicleStatus::ARMING_STATE_STANDBY_ERROR;

            if (this->sitl_ready_)
                this->sendLogInfo("Simulation ready!");
            else
                this->sendLogError("Errors in the simulation detected!");

            this->setHome();

            return;
        }

        // Wait for a bit before retrying
        std::this_thread::sleep_for(std::chrono::milliseconds(constants::DELAY_MILLIS));
    }

    this->sendLogDebug("Status data wait finished");

    // Default to the hypothesis that the simulation has some problem
    this->sitl_ready_ = false;
    this->sendLogError("Errors in the simulation detected!");
}

void UNSCModule::activateOffboardMode(float x, float y, float z, float yaw) {
    cancelTimer(this->offboard_timer_);

    Eigen::Vector3f des_body_pos = this->convertLocalToBody({x, y, z});

    this->sendLogDebug(
        "offboard: ({:.4f}, {:.4f}, {:.4f}) became ({:.4f}, {:.4f}, {:.4f})", x, y, z,
        des_body_pos(0), des_body_pos(1), des_body_pos(2)
    );

    this->offboard_timer_ =
        this->node_->create_wall_timer(this->offboard_period_, [this, des_body_pos, yaw] {
            this->setAndMaintainOffboardMode(
                des_body_pos(0), des_body_pos(1), des_body_pos(2), yaw
            );
        });
}

Eigen::Vector3f UNSCModule::convertLocalToBody(const Eigen::Vector3f& enu_pos) const {
    // Current-axis composition: 1st_rotation * ... * nth_rotation
    Eigen::Matrix3f rot_ENU2NED = rotZ(-90, true) * rotY(180, true);

    Eigen::Vector3f pos_offset_ENU(this->getOffset().data());

    // p_NED = - R_ENU2NED * p_ENU
    Eigen::Vector3f pos_offset_NED = -rot_ENU2NED.transpose() * pos_offset_ENU;
    Eigen::Vector4f hom_offset_NED = pos_offset_NED.homogeneous();

    Eigen::Vector4f hom_enu_pos = enu_pos.homogeneous();

    Eigen::Matrix4f hom_ENU2NED;
    hom_ENU2NED.block(0, 0, 3, 3) = rot_ENU2NED;
    hom_ENU2NED.block(3, 0, 1, 3) << 0, 0, 0;
    hom_ENU2NED.col(3) << hom_offset_NED;

    // The yaw rotation is thought to be along the Z_ENU axis
    // Fixed-axis composition: nth_rotation * ... * 1st_rotation
    // Here the first rotation is the one re-aligning the rotated vehicle to the ENU frame
    Eigen::Vector4f body_pos = hom_ENU2NED * hom_enu_pos;

    return body_pos.head(3);
}

void UNSCModule::setAndMaintainOffboardMode(float x, float y, float z, float yaw) {
    this->sendLogDebug("Offboard to ({:.4f},{:.4f},{:.4f}) yaw:{:.4f}", x, y, z, yaw);

    if (this->offboard_setpoint_counter_ == constants::OFFBOARD_SETPOINT_LIMIT) {
        // Change to Offboard mode after the needed amount of setpoints
        this->sendLogDebug("Changing to Offboard mode!");
        this->signalPublishVehicleCommand(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
            constants::MAVLINK_ENABLE_CUSTOM_MODE, constants::PX4_OFFBOARD_MODE
        );

        this->arm();
    }

    // offboard_control_mode needs to be paired with trajectory_setpoint
    this->signalPublishOffboardControlMode();
    this->signalPublishTrajectorySetpoint(x, y, z, yaw);

    // stop the counter after reaching OFFBOARD_SETPOINT_LIMIT + 1
    if (this->offboard_setpoint_counter_ <= constants::OFFBOARD_SETPOINT_LIMIT) {
        this->offboard_setpoint_counter_++;
    }
}

bool UNSCModule::sendToCommanderUnit(
    uint16_t command, float param1, float param2, float param3, float param4, float param5,
    float param6, float param7
) {
    unsigned int attempt = 1;
    do {
        this->sendLogDebug("Trying to send command {} and to get an ack", command);
        this->signalPublishVehicleCommand(
            command, param1, param2, param3, param4, param5, param6, param7
        );
        attempt++;
    } while ((!this->signalWaitForCommanderAck(command)) &&
             attempt < constants::MAX_SEND_COMMAND_RETRIES);

    if (attempt >= constants::MAX_SEND_COMMAND_RETRIES) {
        this->sendLogWarning("No ack received from the commander unit for command {}", command);
        return false;
    }

    this->sendLogInfo("Command {} sent", command);
    return true;
}
