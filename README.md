# SpartanLIFT (Swarm dePloyment in Autonomous Robot TeleoperAtioNs for Load-Intensive Flight and Transport)

MSc thesis regarding the application of a teleoperated multi-UAV system for cooperative payload transport.

<img alt="Github License" src="https://img.shields.io/github/license/slim71/SpartanLIFT" />

![License details](media/license_info.svg)

---

- [SpartanLIFT (Swarm dePloyment in Autonomous Robot TeleoperAtioNs for Load-Intensive Flight and Transport)](#spartanlift-swarm-deployment-in-autonomous-robot-teleoperations-for-load-intensive-flight-and-transport)
  - [Environment info](#environment-info)
    - [OS](#os)
    - [ROS](#ros)
    - [Gazebo Garden](#gazebo-garden)
  - [Controller library](#controller-library)
  - [Models](#models)
  - [Installation](#installation)
    - [Requirements](#requirements)
    - [Warnings during setup](#warnings-during-setup)
  - [Documentation](#documentation)


## Environment info
### OS
```shell
$ lsb_release -a
No LSB modules are available.
Distributor ID:	Ubuntu
Description:	Ubuntu 22.04.2 LTS
Release:	22.04
Codename:	jammy
```

### ROS
```shell
$ echo $ROS_DISTRO
humble
```
The general **ROS2 underlay** is automatically sourced during each terminal startup in my configuration. I've achieved so simply by
adding `source /opt/ros/humble/setup.bash` to the `~/.bashrc` file. If you don't want to do so, *always remember to manually source it*.

Additionally, at the time of this project's creation, to correctly use **ROS2** and **Gazebo Garden** we need to configure `ros_gz` in the local machine from source: this contains all additional packages needed for the *ROS2-Gazebo integration*.
We can simply clone the repository in a workspace as a ROS2 package. Precise instructions can be found in the [GitHub repository](https://github.com/gazebosim/ros_gz/tree/humble).
Once configured, be sure to also source that **overlay** before using this project.

***Note***: if you don't want to configure `ros_gz`, check if pre-compiled packages are available.

### Gazebo Garden
```shell
$ gz sim --version
Gazebo Sim, version 7.7.0
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

## Installation
To build this project you can easily follow the official guidelines to build and execute ROS2 nodes.
Here's a short step-by-step:
1. `git clone` this project in a local folder
2. Install all dependencies, specified in the [Requirements](#requirements) chapter
3. Source your global ROS2 environment (aka the *underlay*)
4. Build the project using `colcon build`
5. (optional) Execute smoke and unit tests with `colcon test`
6. Source the `install/setup.bash` script (aka the *overlay*)
7. Start the desired node or launch file

### Requirements
For a more specific list, see the output of the ["discover dependencies" task](src/pelican/doc/dependency_list.txt) task.

A simple list:
- [ROS2 Humble](https://docs.ros.org/en/humble/index.html) &rarr; Installed as per official guide
- [colcon](https://colcon.readthedocs.io/en/released/) &rarr; Installed as per official guide (with autocompletion)
- [fmt](https://github.com/fmtlib/fmt) &rarr; Installed from source
- [pugixml](https://pugixml.org/) &rarr; Installed from source
- [PX4](https://github.com/PX4/PX4-Autopilot) &rarr; Installed as per official guide
- eProsima [Micro XRCE-DDS Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent) &rarr; Installed as per official guide

To this you should also add all the other dependencies specified by third-party libraries used by the project.

### Warnings during setup
As per this version, the output of `colcon build` has some warning messages to it. Moreover, I still have some warning myself, which I'll eventually resolve.

An example of output after a complete run, for the sake of completeness, can be found in [this file](doc/warnings.md).

## Documentation
To generate doxygen style documentation, simply move to the root of the project and execute `doxygen doc/Doxyfile`.
Output files will be created in the *doc/html* and *doc/latex* subfolders.
