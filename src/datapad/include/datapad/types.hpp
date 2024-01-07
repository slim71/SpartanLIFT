#ifndef _TYPES_HPP_
#define _TYPES_HPP_

#include "comms/msg/heartbeat.hpp"
#include "comms/msg/poi.hpp"
#include "comms/msg/proposal.hpp"
#include "comms/msg/request_vote_rpc.hpp"
#include "comms/msg/status.hpp"
#include "px4_msgs/msg/failsafe_flags.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/sensor_gps.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_control_mode.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include <boost/circular_buffer.hpp>
#include <chrono>
#include <fmt/core.h>
#include <iostream>
#include <math.h>
#include <optional>
#include <queue>
#include <random>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <string>
#include <vector>

#endif
