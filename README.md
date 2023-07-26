# SpartanLIFT (Swarm dePloyment in Autonomous Robot TeleoperAtioNs for Load-Intensive Flight and Transport)

MSc thesis regarding the application of a teleoperated multi-UAV system for cooperative payload transport.

---

## Environment info
### OS
```bash
slim71@slim71-Ubuntu:~$ lsb_release -a
No LSB modules are available.
Distributor ID:	Ubuntu
Description:	Ubuntu 22.04.2 LTS
Release:	22.04
Codename:	jammy
```

### ROS
```bash
slim71@slim71-Ubuntu:~$ echo $ROS_DISTRO 
humble
```
### Gazebo Garden
```bash
ùslim71@slim71-Ubuntu:~/Documents/git/SpartanLIFT$ gz sim --version
Gazebo Sim, version 7.5.0
Copyright (C) 2018 Open Source Robotics Foundation.
Released under the Apache 2.0 License.
```

---

## Controller library
I initially started with [Ardupilot](https://ardupilot.org/), but I soon had to switch over to [PX4](https://px4.io/) 
because the former was not yet ready to be used in ROS2 projects when I started out.
Among many things, this meant migrating from Gazebo 6 (Ignition) to Gazebo (Garden).

## Models
Drone models and world files downloaded from the [Fuel models page](https://app.gazebosim.org/fuel/models):
- [X3 drone](https://app.gazebosim.org/OpenRobotics/fuel/models/X3%20UAV%20Config%201) by **OpenRobotics**
- [X4 drone](https://app.gazebosim.org/OpenRobotics/fuel/models/X4%20UAV%20Config%201) by **OpenRobotics**
- [Tugbot depot](https://app.gazebosim.org/MovAi/fuel/worlds/tugbot_depot) by **MovAi**
- [Tugbot warehouse](https://app.gazebosim.org/MovAi/fuel/worlds/tugbot_warehouse) by **MovAi**
I modified them in order to use it for my project.

## External libraries
- pugixml from source + make + sudo make install
- ccache

## Installation

1. PX4
2. px4_msgs
3. px4_ros_com
4. microxrcedds_agent
5. my packages

TODO: order of colcon build?

document setuptools warning/problem for python packages

// Found this about timeouts:
// The broadcast time should be an order of magnitude less than the election timeout so that leaders can
// reliably send the heartbeat messages required to keep followers from starting elections; given the randomized approach 
// used for election timeouts, this inequality also makes split votes unlikely. The election timeout should be
// a few orders of magnitude less than MTBF so that the system makes steady progress

rclcpp::create_wall_timer is a factory function that creates an object of a class that derives from rclcpp::TimerBase and uses the wall clock as the timer's time source.