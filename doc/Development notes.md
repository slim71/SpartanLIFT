# Development notes
## Raft
From the official [Raft paper](https://raft.github.io/raft.pdf):
> The broadcast time should be an order of magnitude less than the election timeout so that leaders can
> reliably send the heartbeat messages required to keep followers from starting elections; given the randomized approach
> used for election timeouts, this inequality also makes split votes unlikely. The election timeout should be
> a few orders of magnitude less than MTBF so that the system makes steady progress

Theoretically, the leader keeps trying to send entries indefinitely in Raft. Here we try for a max amount of times

Raft uses randomized election timeouts to ensure that split votes are rare and that they are resolved quickly

---
## Gazebo
Following the [official guide about the migration from Gazebo classic](https://github.com/gazebosim/gz-sim/blob/gz-sim7/tutorials/migration_sdf.md#path-relative-to-an-environment-variable), I've noticed I had the wrong environmental variable setup for the models loading. I've fixed that and now, *after sourcing the local overlay,* Gazebo can successfully load each model's meshes with no problems.
As of this note creation, the correct variable used by Gazebo is `GZ_SIM_RESOURCE_PATH`.

---
## PX4 topics

Mapped in [this file](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml).
All messages are already defined in PX4.
List:
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

PX4 accepts VehicleCommand messages only if their target_system field is zero (broadcast
communication) or coincides with MAV_SYS_ID. In all other situations, the messages are
ignored. For example, if you want to send a command to your third vehicle, which has
px4_instance=2, you need to set target_system=3 in all your VehicleCommand messages.

Note about the VehicleCommandAck:
https://github.com/PX4/PX4-Autopilot/issues/21430#issuecomment-1497484098

The check ((1u << status->nav_state) != 0) into UNSCModule::runPreChecks() was taken directly from the PX4 commander.
It checks whether the current nav_state of the agent is set or not.

PX4 requires that the vehicle is already receiving OffboardControlMode messages before it will arm in offboard mode, or before it will switch to offboard mode when flying. In addition, PX4 will switch out of offboard mode if the stream rate of OffboardControlMode messages drops below approximately 2Hz.

TODO: delete useless couts from PX4

### Offboard control
https://docs.px4.io/main/en/flight_modes/offboard.html
PX4 requires that the external controller provides a continuous 2Hz "proof of life" signal, by streaming any of the supported MAVLink setpoint messages **or** the ROS 2 OffboardControlMode message.
All values are interpreted in NED (Nord, East, Down); unit is [m].

---
## Commands
colcon build --packages-select pelican --cmake-args -DDEBUG_MODE=1
colcon build --packages-select pelican --cmake-args -DBUILD_TESTING=1
colcon build --packages-select pelican --cmake-args -DPEDANTIC_MODE=1
colcon test --packages-select pelican --event-handlers=console_cohesion+
colcon test-result --all
gdbtui build/pelican/pelican
gdbtui install/pelican/lib/pelican/pelican_test
gdbtui --args build/pelican/pelican --ros-args --params-file src/pelican/config/copter_test.yaml --log-level debug
handle SIGINT noprint nostop pass
ros2 run datapad datapad --ros-args --log-level debug
ros2 run pelican pelican --ros-args --params-file src/pelican/config/copter_test.yaml --log-level debug
ros2 run --prefix 'gdbtui -ex run --args' pelican pelican --ros-args --params-file src/pelican/config/copter1.yaml
ros2 run --prefix 'valgrind --tool=callgrind' pelican pelican --ros-args --params-file src/pelican/config/copter1.yaml
ros2 launch odst ros_agents.launch.py loglevel:=debug
MicroXRCEAgent udp4 -p 8888
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='0,1' PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i 1
grep "\[Agent 1|" launch.log > agent1.log && grep "\[Agent 2|" launch.log > agent2.log && grep "\[Agent 3|" launch.log > agent3.log && grep "\[Agent 4|" launch.log > agent4.log && grep "\[Agent 5|" launch.log > agent5.log

TODO: [here](https://docs.px4.io/main/en/sim_gazebo_gz/) it says:
The environmental variable PX4_GZ_MODEL has been deprecated and its functionality merged into PX4_SIM_MODEL.
--> change it.

---
## Mission useful stuff
If set, a multi-rotor vehicle will yaw to face the Heading value specified in the target waypoint (corresponding to MAV_CMD_NAV_WAYPOINT.param4).
If Heading has not been explicitly set for the target waypoint (param4=NaN) then the vehicle will yaw towards a location specified in the parameter MPC_YAW_MODE. By default this is the next waypoint.

PX4 runs some basic sanity checks to determine if a mission is feasible when it is uploaded, and when the vehicle is first armed. If any of the checks fail, the user is notified and it is not possible to start the mission.

PX4 expects to follow a straight line from the previous waypoint to the current target (it does not plan any other kind of path between waypoints - if you need one you can simulate this by adding additional waypoints).

Vehicles switch to the next waypoint as soon as they enter the acceptance radius.
For a multi-rotor drones, the acceptance radius is tuned using the parameter NAV_ACC_RAD.

During mission execution this will cause the vehicle to ascend vertically to the minimum takeoff altitude defined in the MIS_TAKEOFF_ALT parameter, then head towards the 3D position defined in the mission item.

---
## General notes
Can't use this in the tests
```cpp
EXPECT_THAT(
      [this]() { this->core_.setupPublisher(); },
      ThrowsMessage<std::runtime_error>(HasSubstr(EXTERNAL_OFF))
);
```
because in **ament_cmake_gmock** at the 'humble' tag `ThrowsMessage` (as other functions) is not defined


CHECK: ROI as fleet radius?

Considering that external functionalities are not active if the main module is not present, everything can throw an error if node_ is not set

TODO: add ros_gz_sim as package?
TODO: create Micro-XRCE as own package

'make' in PX4 folder before launch file after installation

TODO: check eigen is in requirements
TODO: change nodes name
CHECK: gazebo topics, maybe from unused plugins
TODO: add models to pelican from PX4, so both can be used to inspect them
TODO: explictly say about `source /opt/ros/humble/setup.bash` and `~/Documents/git/ros_gz_workspace/install/setup.bash`
TODO: what about ros_gz_workspace?

---
## Launch file notes
The optional dictionary `launch_arguments` for `IncludeLaunchDescription` is only valid to pass arguments to a target launch file that defines arguments with `LaunchConfiguration`s.

For node executables, use a list `arguments`, depending on the executable's source.

---
To correctly spawn everything in the same window with `gnome-terminal`, I had to use a temporary file. That's because, even when the `--tab` option is used, launching it from another application always opens a new window instead of a tab.

As reference, see [this comment](https://github.com/GNS3/gns3-gui/issues/3449#issuecomment-1532133451) on GitHub.

---
The empty file in the temp folder is needed because it will be copied over to the share folder of the package and used to structure a bash script.

---
The overall launch with all applications is mostly done via Bash to have more flexibility. \
As an example, starting Gazebo as done in other files (including the launch description of the `ros_gz_sim` package) is equally valid,
but this prevented me from changing the content of the `GZ_SIM_RESOURCE_PATH` environment variable to allow the simulator to find all PX4 models/worlds. Instead of doing it in the `.bashrc` file of the local machine, I opted to highlight this need in the project itself.

---
When launching Gazebo, an error usually appears on screen
```shell
libEGL warning: egl: failed to create dri2 screen
libEGL warning: egl: failed to create dri2 screen
```
This is only a warning and does not affect Gazebo's behavior.\
More info [here](https://github.com/gazebosim/gz-rendering/issues/587).
