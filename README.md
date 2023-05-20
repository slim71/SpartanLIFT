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
### Ignition Gazebo
```bash
slim71@slim71-Ubuntu:~$ ign gazebo --version
Ignition Gazebo, version 6.11.0
Copyright (C) 2018 Open Source Robotics Foundation.
Released under the Apache 2.0 License.
```

---

## Models
Drone models and world files found in the [Fuel models page](https://app.gazebosim.org/fuel/models):
- [X3 drone](https://app.gazebosim.org/OpenRobotics/fuel/models/X3%20UAV%20Config%201) by **OpenRobotics**
- [X4 drone](https://app.gazebosim.org/OpenRobotics/fuel/models/X4%20UAV%20Config%201) by **OpenRobotics**
- [Tugbot depot](https://app.gazebosim.org/MovAi/fuel/worlds/tugbot_depot) by **MovAi**
- [Tugbot warehouse](https://app.gazebosim.org/MovAi/fuel/worlds/tugbot_warehouse) by **MovAi**