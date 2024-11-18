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
## ROS2
Callback for subscribers and timers need to be placed in a callback group, depending on the control we want
to achieve on their executions.
Further details can be found in the [ROS2 docs](), but here's a quick sum-up.

For the interaction of an individual callback with itself:
- if it ***should*** be executed in parallel to itself --> **Reentrant Callback group**
- if it ***should not*** be executed in parallel to itself --> **Mutually Exclusive Callback group**

For the interaction of different callbacks with each other:
- if they ***should not*** be executed in parallel --> ***same*** **Mutually Exclusive Callback group**
- if they ***should*** be executed in parallel
  - with *no overlap* of the individual callbacks --> ***different*** **Mutually Exclusive Callback groups**
  - with *overlap* of the individual callbacks --> **Reentrant Callback Group**

Note that if the group for a callback is not explicitly specified, the default one is used: that's shared across
all callbacks with a non-specified group.

To help identify ROS2 communication channels used in the code, here's a quick reference.
1. *Actions*
   1. **TeleopData**
      - Client node: **Datapad**
        - request function: `Datapad::teleopTaskClient`
        - response analyzer: `Datapad::analyzeTeleopDataResponse`
        - feedback handler: `Datapad::parseTeleopDataFeedback`
        - final result handler: `Datapad::processFleetLeaderCommandAck`
      - Server node: **Pelican** (leader)
        - goal receiver: `Pelican::handleTeleopDataGoal`
        - acceptance handler: `Pelican::handleAcceptedTeleopData`
        - goal execution: `Pelican::rogerWillCo`
        - feedback generator: `Pelican::handleCommandReception`
        - cancellation handler: `Pelican::handleTeleopDataCancellation`
2. *Services*
   1. **FleetInfo**
      - First exchange type: *Target position*
        - Server node: **Pelican** (leader)
          - server function: `Pelican::targetNotification`
        - Client node: **Pelican** (follower)
          - request function: `Pelican::rendezvousFleet`
          - result handler function: `Pelican::processLeaderResponse`
      - Second exchange type: *Neighbor desired position*
        - Server node: **Pelican**
          - server function: `Pelican::shareDesiredPosition`
        - Client node: **Pelican**
          - request function: `Pelican::askDesPosToNeighbor`
          - result handler function: `Pelican::storeNeighborDesPos`
   2. **CargoPoint**
      - Server node: **Cargo**
        - server function: `Cargo::shareCargoPosition`
      - Client node: **Datapad**
        - request function: `Datapad::askForCargoPoint`
        - result handler function: `Datapad::storeCargoPoint`
   3. **CargoLinkage**
      - Server node: **Cargo**
        - server function: `Cargo::notifyAttachment`
      - Client node: **Pelican**
        - request function: `Pelican::cargoAttachment`
        - result handler function: `Pelican::checkCargoAttachment`
   4. **FormationReached**
      - Server node: **Pelican** (leader)
        - server function: `Pelican::recordAgentInFormation`
      - Client node: **Pelican** (follower)
        - request function: `Pelican::notifyAgentInFormation`
        - result handler function absent, since the response is empty

Logs cannot be automatically printed in different files, at the moment, as this would require an ad-hoc implementation
of the `rcl_logging_interface` (as stated [here](https://robotics.stackexchange.com/a/104433/30956)).
I'll just manually do it; perhaps that's a thing for the future...

Some constants declared in constants.hpp could have been taken from the PX4 package, but I prefer to keep them entirely separated
and independent from one another.

### Launch files
The helper functions to print debug info of arguments and LaunchConfigurations only work if those were added to the LaunchDescription **before** the OpaqueFunction handling the printing.

### ros_gz package
I've created a supplemental workspace in the project, aimed at sustaining a fork of the `ros_gz` package.

Even though I initially used the `humble` branch of the original project as is, when dealing with a cargo which has to move together with the leader of the quadcopters fleet, I've noticed that the `SetEntityPose` service was not supported yet by `ros_gz_bridge`.

Since I think I'll need that in order to easily move the box around, I've forked the repository and manually picked some commits found in another pre-existing fork: [UoA-Cares/ros_gz](https://github.com/UoA-CARES/ros_gz). I could not easily cherry-picked those changes because they're made on the `ros2` branch, which has major differences from the one I need to use, due to my configuration.

At the present state, this workspace should be the first one being built and source when setting everything up. For ease of use, I've added the sourcing of this overlay in my `.bashrc` file.

---
## Gazebo
Following the [official guide about the migration from Gazebo classic](https://github.com/gazebosim/gz-sim/blob/gz-sim7/tutorials/migration_sdf.md#path-relative-to-an-environment-variable), I've noticed I had the wrong environmental variable setup for the models loading. I've fixed that and now, *after sourcing the local overlay,* Gazebo can successfully load each model's meshes with no problems.
As of this note creation, the correct variable used by Gazebo is `GZ_SIM_RESOURCE_PATH`.

I also need some topics published by Gazebo plugins, so I have introduced `ros_gz_bridge` in the launch file. This allows the porting of messages and services from Gazebo to ROS2 and viceversa.

### Cargo model
Since at the time of this development the `gravity` tag was not available per-model (well, it is but it's also not actually parsed in Gazebo Garden), I've turned to using the `static` tag instead. This would hinder *any* movement of the model (not only prevent the cargo model to fall due to the gravitational force), but since there won't be anything else here, apart from the manual calls to the `set_pose` gz service, that's good enough.
See [issue #504](https://github.com/gazebosim/gz-sim/issues/504) for more details and to follow related developments.

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

PX4 accepts VehicleCommand messages only if their target_system field is zero (broadcast communication) or coincides with MAV_SYS_ID. In all other situations, the messages are ignored. For example, if you want to send a command to your third vehicle, which has
px4_instance=2, you need to set target_system=3 in all your VehicleCommand messages.

Note about the VehicleCommandAck:
https://github.com/PX4/PX4-Autopilot/issues/21430#issuecomment-1497484098

The check ((1u << status->nav_state) != 0) into UNSCModule::runPreChecks() was taken directly from the PX4 commander.
It checks whether the current nav_state of the agent is set or not.

### Offboard control
https://docs.px4.io/main/en/flight_modes/offboard.html
PX4 requires that the external controller provides a continuous 2Hz "proof of life" signal, by streaming any of the supported MAVLink setpoint messages **or** the ROS 2 OffboardControlMode message.
All values are interpreted in NED (Nord, East, Down); unit is [m].

The OffboardControlMode is required in order to inform PX4 of the type of offboard control being used. Here we're only using position control, so the position field is set to true and all the other fields are set to false.

PX4 requires that the vehicle is already receiving OffboardControlMode messages before it will arm in offboard mode, or before it will switch to offboard mode when flying. In addition, PX4 will switch out of offboard mode if the stream rate of OffboardControlMode messages drops below approximately 2Hz.

Non-NAN values are used, NAN values are ignored.
All values are interpreted in NED (Nord, East, Down) coordinate system and the units are [m], [m/s] and [m/s^2] for position, velocity and acceleration, respectively.

### Obstacle distance
~~Distance-based obstacle avoidance based on the work by [dakejahl](https://github.com/dakejahl) in [this PR](https://github.com/PX4/PX4-Autopilot/pull/22418/). The PX4 Offboard collision prevention accounts for max 36 sectors of 10 degrees each, but the status it is implemented in requires a companion computer, which is not ideal in this project. To overcome this, I've tried implementing a simple obstacle avoidance based on a laser scanner sensor. The laser scanner handling is taken from [this fork's PR](https://github.com/PX4/PX4-Autopilot/pull/22418). Based on how it's done and on the data generated by Gazebo, the total area covered by the sensor is around 270 degrees, leaving a 90 degrees "blind" spot in the quadcopter's rear end (so covering the range [-135°; 135°]; note that the forward direction of the drone is the x-axis). In practice, only the first 27 elements of the distance vector in the ROS2 topic are used, since everything is downsampled. To map an obstacle presence, these sectors/elements are grouped into three sub-ranges, each of them defining a relative direction in which a obstacle can be detected (left, front, right). One unit corresponds to 1cm. I've personally twicked the Lidar model used because the OpenRobotics' one was causing errors in Gazebo when a simulation was launched.~~

Introducing a 2D laser sensor for each multicopter was impeding the simulation: the real-time factor dropped to about 10% when introduced, and nothing worked as expected.
Since all copters will share their position with the others when achieving formation control, I'd leverage that information starting now to also develop a simple obstacle avoidance algorithm.

### Sensors
Best result for an isolated run with emesent_hovermap: no simulated baro (`SENS_EN_BAROSIM=0`) and air pressure sensor coded in the sdf model file, with `update_rate=50, <noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise>`
Best result for multi-vehicle: only simulated barometer.

---
## Commands
px4 numbers from
  /home/slim71/Documents/git/SpartanLIFT/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
name from the model in
  /home/slim71/Documents/git/SpartanLIFT/PX4-Autopilot/Tools/simulation/gz/models/

before compiling everything, needed by ros_gz_sim: export GZ_VERSION=garden

colcon build --packages-select pelican --cmake-args -DDEBUG_MODE=1
colcon build --packages-select pelican --cmake-args -DBUILD_TESTING=1
colcon build --packages-select pelican --cmake-args -DPEDANTIC_MODE=1
colcon test --packages-select pelican --event-handlers=console_cohesion+
colcon test-result --all
gdbtui build/pelican/pelican
gdbtui install/pelican/lib/pelican/pelican_test
gdbtui --args build/pelican/pelican --ros-args --params-file src/pelican/config/copter_test.yaml --log-level debug
handle SIGINT noprint nostop pass
thread apply all bt   |   t a a bt
ros2 run datapad datapad --ros-args --log-level debug
ros2 run pelican pelican --ros-args --params-file src/pelican/config/copter_test.yaml --log-level debug
ros2 run pelican pelican --ros-args -p model:="/home/slim71/Documents/git/SpartanLIFT/src/pelican/models/x500/model.sdf" -p id:=1 -p roi:=2.0 --log-level debug
ros2 run pelican pelican --ros-args --remap __node:=pelican_2 --params-file src/pelican/config/fleet.yaml --log-level debug
ros2 run --prefix 'gdbtui -ex run --args' pelican pelican --ros-args --params-file src/pelican/config/copter1.yaml
ros2 run --prefix 'valgrind --tool=callgrind' pelican pelican --ros-args --params-file src/pelican/config/copter1.yaml
ros2 launch odst ros_agents.launch.py loglevel:=debug
MicroXRCEAgent udp4 -p 8888
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='0,1' PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i 1
grep "\[Agent 1|" launch.log > agent1.log && grep "\[Agent 2|" launch.log > agent2.log && grep "\[Agent 3|" launch.log > agent3.log && grep "\[Agent 4|" launch.log > agent4.log && grep "\[Agent 5|" launch.log > agent5.log
ros2 run ros_gz_bridge parameter_bridge /world/empty/set_pose@ros_gz_interfaces/srv/SetEntityPose
ros2 service call /world/empty/set_pose ros_gz_interfaces/srv/SetEntityPose "{entity: {name: 'sphere'}, pose: {position: {x: 4, y: 3, z: 2}}}"

---
## Mission useful stuff
If set, a multi-rotor vehicle will yaw to face the Heading value specified in the target waypoint (corresponding to MAV_CMD_NAV_WAYPOINT.param4).
If Heading has not been explicitly set for the target waypoint (param4=NaN) then the vehicle will yaw towards a location specified in the parameter MPC_YAW_MODE. By default this is the next waypoint.

PX4 runs some basic sanity checks to determine if a mission is feasible when it is uploaded, and when the vehicle is first armed. If any of the checks fail, the user is notified and it is not possible to start the mission.

PX4 expects to follow a straight line from the previous waypoint to the current target (it does not plan any other kind of path between waypoints - if you need one you can simulate this by adding additional waypoints).

Vehicles switch to the next waypoint as soon as they enter the acceptance radius.
For a multi-rotor drones, the acceptance radius is tuned using the parameter NAV_ACC_RAD.

During mission execution this will cause the vehicle to ascend vertically to the minimum takeoff altitude defined in the MIS_TAKEOFF_ALT parameter, then head towards the 3D position defined in the mission item.

When an agents stops responding and the network needs to be downsized, we simply put a NAN value in the vector used to store all agents positions. Since it is reasonable to think that a network size does not change dramatically during normal operations, we simply don't worry about memory restrictions and usage optimizations (aka we do not resize the vector).

The Gazebo odometry and the PX4 local positions have about 0.2m off.
Because of this disalignment, the computations made when the copters are in offboard mode. We started checking if the distance between a copter and the latest setpoint is less than 0.2m on each axis and if it was, we would send another velocity setpoint for the offboard mode to track, in order to close in to the point.
Unfortunately, since the PX4 estimator judges that the copter is already at the desired position, no further movement is made and the offset remains.
*For now*, we've switched to a **0.4m threshold**.

That's basically the same reason we needed a manual height compensation: Gazebo's data and the PX4 estimation were not equal, so we had to make sure all copters were hovering around the same height for this application.

All algorithms do not directly handle the height (z axis position): this is handled separately by a periodic height compensation algorithm which only focuses on that and tries to stabilize each agent to the desired altitude.

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

Considering that external functionalities are not active if the main module is not present, everything can throw an error if node_ is not set.

'make' in PX4 folder before launch file after installation

PX4 and mavlink support global setpoints, but only with GPS coordinates. Since we're set inside a warehouse, the application would most probably use some sort of inside positioning system instead. The conversion from this to the vehicles' body frames will have to be made manually then.

Seems like the EKF2 is initialized in {0,0,0} even when using
`PX4_SYS_AUTOSTART={code} PX4_GZ_MODEL_POSE='{x},{y}' PX4_SIM_MODEL={model} ./build/px4_sitl_default/bin/px4 -i {count+1}`
so for now the global position is not used

~~Displacement-based control: agents control displacements to achieve formation (specified by desired displacements wrt a global coord. system); each agent can sense neighbors' position wrt the global coord. system~~
distance-based

Beware about the way NAN is treated and how to check for it, depending on the compiler used. Try not to use the `--fast-math` flag, if possible.
Check [this discussion on SO]() for further details.

---
## Launch file notes
The optional dictionary `launch_arguments` for `IncludeLaunchDescription` is only valid to pass arguments to a target launch file that defines arguments with `LaunchConfiguration`s.

For node executables, use a list `arguments`, depending on the executable's source.

To correctly spawn everything in the same window with `gnome-terminal`, I had to use a temporary file. That's because, even when the `--tab` option is used, launching it from another application always opens a new window instead of a tab.

As reference, see [this comment](https://github.com/GNS3/gns3-gui/issues/3449#issuecomment-1532133451) on GitHub.

The empty file in the temp folder is needed because it will be copied over to the share folder of the package and used to structure a bash script.

The overall launch with all applications is mostly done via Bash to have more flexibility. \
As an example, starting Gazebo as done in other files (including the launch description of the `ros_gz_sim` package) is equally valid,
but this prevented me from changing the content of the `GZ_SIM_RESOURCE_PATH` environment variable to allow the simulator to find all PX4 models/worlds. Instead of doing it in the `.bashrc` file of the local machine, I opted to highlight this need in the project itself.

When launching Gazebo, an error usually appears on screen
```shell
libEGL warning: egl: failed to create dri2 screen
libEGL warning: egl: failed to create dri2 screen
```
This is only a warning and does not affect Gazebo's behavior.\
More info [here](https://github.com/gazebosim/gz-rendering/issues/587).

---
## TODOs

TODO: fork of px4_msgs for the queue change
TODO: script to gather Gazebo "create" service's response?
TODO: different threads for role handling and actions?
TODO: change datapad checks if agents relaunched? crash should not happen in theory
CHECK: do not accept formation task if rendezvous not finished
