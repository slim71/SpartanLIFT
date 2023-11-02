# Development notes
From the official [Raft paper](https://raft.github.io/raft.pdf):
> The broadcast time should be an order of magnitude less than the election timeout so that leaders can
> reliably send the heartbeat messages required to keep followers from starting elections; given the randomized approach
> used for election timeouts, this inequality also makes split votes unlikely. The election timeout should be
> a few orders of magnitude less than MTBF so that the system makes steady progress

---
// TODO: consider mutex or interrupt for consecutive changes in role

---
Following the [official guide about the migration from Gazebo classic](https://github.com/gazebosim/gz-sim/blob/gz-sim7/tutorials/migration_sdf.md#path-relative-to-an-environment-variable), I've noticed I had the wrong environmental variable setup for the models loading. I've fixed that and now, *after sourcing the local overlay,* Gazebo can successfully load each model's meshes with no problems.
As of this note creation, the correct variable used by Gazebo is `GZ_SIM_RESOURCE_PATH`.

---
### PX4 topics

Mapped in [this file](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml).

All messages are already defined in PX4.

/px4_{id}/fmu/in
    /obstacle_distance
    /offboard_control_mode
    /onboard_computer_status
    /sensor_optical_flow
    /telemetry_status
    /trajectory_setpoint
    /vehicle_attitude_setpoint
        /vehicle_command
    /vehicle_mocap_odometry
    /vehicle_rates_setpoint
    /vehicle_trajectory_bezier
    /vehicle_trajectory_waypoint
    /vehicle_visual_odometry

/px4_{id}/fmu/out
            /failsafe_flags
    /position_setpoint_triplet
    /sensor_combined
    /timesync_status
            /vehicle_attitude
        /vehicle_control_mode
        /vehicle_global_position
            /vehicle_gps_position
        /vehicle_local_position
        /vehicle_odometry
        /vehicle_status

---
### Commands
ros2 run pelican pelican --ros-args --params-file src/pelican/config/copter_test.yaml --log-level debug
colcon build --packages-select pelican --cmake-args -DDEBUG_MODE=1
gdbtui build/pelican/pelican
ros2 run --prefix 'gdbtui -ex run --args' pelican pelican --ros-args --params-file src/pelican/config/copter1.yaml
ros2 run --prefix 'valgrind --tool=callgrind' pelican pelican --ros-args --params-file src/pelican/config/copter1.yaml

---
https://docs.px4.io/main/en/flight_modes/offboard.html

---
PX4 accepts VehicleCommand messages only if their target_system field is zero (broadcast
communication) or coincides with MAV_SYS_ID. In all other situations, the messages are
ignored. For example, if you want to send a command to your third vehicle, which has
px4_instance=2, you need to set target_system=3 in all your VehicleCommand messages.

---
Note about the VehicleCommandAck:
https://github.com/PX4/PX4-Autopilot/issues/21430#issuecomment-1497484098
