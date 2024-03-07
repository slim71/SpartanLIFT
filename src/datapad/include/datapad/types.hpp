#ifndef _TYPES_HPP_
#define _TYPES_HPP_

#include "comms/msg/poi.hpp"
#include "comms/srv/teleop_data.hpp"
#include <chrono>
#include <fmt/core.h>
#include <iostream>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <string>

namespace constants {
    static constexpr unsigned int SEARCH_LEADER_STEP_SECS = 1;
    static constexpr unsigned int MAX_SEARCH_TIME_SECS = 10;
    static constexpr unsigned int SETUP_TIME_SECS = 1;
    static constexpr unsigned int QOS_HISTORY_AMOUNT = 5;
    static constexpr unsigned int SERVICE_FUTURE_WAIT_SECS = 1;
    static constexpr unsigned int HOMEPAGE_LOOP_WAIT_SECS = 10;
    // TODO: use aggregates
    static constexpr bool PRESENCE_FLAG_ON = true;
    static constexpr bool PRESENCE_FLAG_OFF = false;
    static constexpr bool TAKEOFF_FLAG_ON = true;
    static constexpr bool TAKEOFF_FLAG_OFF = false;
    static constexpr bool LANDING_FLAG_ON = true;
    static constexpr bool LANDING_FLAG_OFF = false;
    static constexpr bool EXTRACTION_FLAG_ON = true;
    static constexpr bool EXTRACTION_FLAG_OFF = false;
    static constexpr bool DROPOFF_FLAG_ON = true;
    static constexpr bool DROPOFF_FLAG_OFF = false;
} // namespace constants

#endif
