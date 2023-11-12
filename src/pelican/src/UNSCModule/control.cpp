#include "PelicanModule/pelican.hpp"
#include "UNSCModule/unsc.hpp"

// #include "px4_custom_mode.h" // TODO: whole header needed?

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

// Pitch| Empty| Empty| Yaw| Latitude| Longitude| Altitude|
void UNSCModule::takeoff(unsigned int height) {
    /*
        in case I'll nee to extend the functionality
        int attempts = 0;

        std::optional<px4_msgs::msg::VehicleGlobalPosition> pos;
        std::optional<px4_msgs::msg::VehicleOdometry> odo;
        // Continue checking for a bit, if program is not stopped
        while (this->getRunningStatus() && attempts < MAX_DATA_ATTEMPTS) {
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
        if (!this->getRunningStatus())
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
    */
    if (height > 0) {
        this->signalPublishVehicleCommand(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, NAN, NAN, NAN, NAN, NAN, NAN,
            height
        );
    } else {
        this->signalPublishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF);
    }

    if (waitForAck(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF)) {
        this->arm();
    }
}

// Empty| Empty| Empty| Yaw| Latitude| Longitude| Altitude|
void UNSCModule::land() {
    this->signalPublishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
}

// Use current?| Empty| Empty| Empty| Latitude| Longitude| Altitude|
void UNSCModule::setHome() {
    this->signalPublishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_HOME,
        1.0 // TODO: find or define constant
    );

    this->sendLogInfo("Set home sent");
}

bool UNSCModule::waitForAck(uint16_t cmd) {
    auto start_time = this->gatherTime().nanoseconds() / 1000000; // [ms]
    this->sendLogDebug("ACK wait started at timestamp {}", start_time);

    // Check for a bit, in case the ACK has not been received yet
    while (this->gatherTime().nanoseconds() / 1000000 - start_time < 100) {
        std::optional<px4_msgs::msg::VehicleCommandAck> ack = this->gatherAck();

        if (ack) { // at least one ACK received
            this->sendLogDebug(
                "Stored ACK found! cmd:{} timestamp:{}", ack->command, ack->timestamp
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

void UNSCModule::runPreChecks() {
    // Delete wall timer to have it set off only once
    cancelTimer(this->starting_timer_);

    this->signalPublishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_RUN_PREARM_CHECKS);

    auto start_time = this->gatherTime().nanoseconds() / 1000000; // [ms]
    this->sendLogDebug("Status data wait started at timestamp {}", start_time);

    // Check for a bit, in case the ACK has not been received yet
    while (this->gatherTime().nanoseconds() / 1000000 - start_time < 100) {
        std::optional<px4_msgs::msg::VehicleStatus> status = this->gatherStatus();

        if (status) { // at least one status received
            this->sendLogInfo(
                "Stored status message found! timestamp:{} arming_state:{} nav_state:{} "
                "pre_flight_checks_pass:{} failsafe:{}",
                status->timestamp, status->arming_state, status->nav_state,
                status->pre_flight_checks_pass, status->failsafe
            );

            // Check whether the agent in the simulation is good to go
            this->sitl_ready_ =
                status->pre_flight_checks_pass && !status->failsafe &&
                ((1u << status->nav_state) != 0) && // taken from the PX4 commander
                status->arming_state < px4_msgs::msg::VehicleStatus::ARMING_STATE_STANDBY_ERROR;

            if (this->sitl_ready_)
                this->sendLogInfo("Simulation ready!");
            else
                this->sendLogError("Errors in the simulation detected!");

            this->setHome();

            // TODO: delete
            this->starting_timer_ = this->node_->create_wall_timer(
                this->briefing_time_,
                [this] {
                    this->takeoff();
                },
                this->gatherReentrantGroup()
            );

            return;
        }

        // Wait for a bit before retrying
        std::this_thread::sleep_for(std::chrono::microseconds(10000));
    }

    this->sendLogDebug("Status data wait finished");

    // Default to the hypothesis that the simulation has some problem
    this->sitl_ready_ = false;
    this->sendLogError("Errors in the simulation detected!");
}

// Empty| Empty| Empty| Empty| Empty| Empty| Empty|
void UNSCModule::returnToLaunchPosition() {
    this->signalPublishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
    );

    this->sendLogInfo("ReturnToLaunch sent");
}

void UNSCModule::setAndMaintainOffboardMode(float x, float y, float z, float yaw) {
    if (this->offboard_setpoint_counter_ == 10) {
        // Change to Offboard mode after 10 setpoints
        this->sendLogDebug("Changing to Offboard mode!");
        this->signalPublishVehicleCommand(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6
        ); // CHECK: can use px4 constants? others?
    }

    // offboard_control_mode needs to be paired with trajectory_setpoint
    this->signalPublishOffboardControlMode();

    // stop the counter after reaching 11
    if (this->offboard_setpoint_counter_ < 11) {
        this->offboard_setpoint_counter_++;
    }
}

void UNSCModule::setPositionMode() {
    this->signalPublishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_HOME, 1, 0);

    this->sendLogInfo("Set position mode sent");
}

/* void UNSCModule::prepareMission() {
    std::string connection_url {"udp://"};
        std::unique_ptr<mavsdk::Mission> _mission{};
        Mission::MissionPlan mission_plan {};
        mission_plan.mission_items.push_back(create_mission_item({20.0, 0.}, mission_options, ct));
        mission_plan.mission_items.push_back(create_mission_item({20.0, 20.0}, mission_options,
ct)); mission_plan.mission_items.push_back(create_mission_item({0., mission_options.leg_length_m},
mission_options, ct));

        _mission->set_return_to_launch_after_mission(mission_options.rtl_at_end);

        REQUIRE(_mission->upload_mission(mission_plan) == Mission::Result::Success);
}

void UNSCModule::missionStart() {
    this->signalPublishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
        1, PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_MISSION
    );
}

void UNSCModule::setAcceptanceRadius() { // only for missions
    this->signalPublishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_HOME,
        1.0 // TODO: find or define constant
    );

    this->sendLogInfo("Set ROI sent");
}

Mission::MissionItem  UNSCModule::createMissionItem(
            const CoordinateTransformation::LocalCoordinate &local_coordinate,
            const MissionOptions &mission_options,
            const CoordinateTransformation &ct) {
        auto mission_item = Mission::MissionItem{};
        const auto pos_north = ct.global_from_local(local_coordinate);

        mission_item.latitude_deg = pos_north.latitude_deg;
        mission_item.longitude_deg = pos_north.longitude_deg;
        mission_item.relative_altitude_m = mission_options.relative_altitude_m;
        mission_item.is_fly_through = mission_options.fly_through;

        return mission_item;
} */
