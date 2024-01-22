#ifndef _TYPES_HPP_
#define _TYPES_HPP_

#include "comms/msg/poi.hpp"
#include "comms/srv/fleet_info_exchange.hpp"
#include <chrono>
#include <fmt/core.h>
#include <iostream>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <string>

namespace constants {
    static constexpr int SEARCH_LEADER_STEP = 1; // [s]
    static constexpr int MAX_SEARCH_TIME = 10;   // [s]
    static constexpr bool PRESENCE_FLAG_ON = true;
    static constexpr bool PRESENCE_FLAG_OFF = false;
    static constexpr bool TAKEOFF_FLAG_ON = true;
    static constexpr bool TAKEOFF_FLAG_OFF = false;
    static constexpr bool LANDING_FLAG_ON = true;
    static constexpr bool LANDING_FLAG_OFF = false;
    static constexpr unsigned int QOS_HISTORY_AMOUNT = 5;
    static constexpr unsigned int SETUP_TIME_SECS = 1;
} // namespace constants

#endif
