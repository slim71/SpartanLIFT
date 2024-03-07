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

class Flags {
    public:
        explicit Flags();

        Flags& SetPresence();
        Flags& SetTakeoff();
        Flags& SetLanding();
        Flags& SetRetrieval();
        Flags& SetDropoff();

        bool GetPresence() const;
        bool GetTakeoff() const;
        bool GetLanding() const;
        bool GetRetrieval() const;
        bool GetDropoff() const;

    private:
        bool presence = false;
        bool takeoff = false;
        bool landing = false;
        bool retrieval = false;
        bool dropoff = false;
};

namespace constants {
    static constexpr unsigned int SEARCH_LEADER_STEP_SECS = 1;
    static constexpr unsigned int MAX_SEARCH_TIME_SECS = 10;
    static constexpr unsigned int SETUP_TIME_SECS = 1;
    static constexpr unsigned int QOS_HISTORY_AMOUNT = 5;
    static constexpr unsigned int SERVICE_FUTURE_WAIT_SECS = 1;
    static constexpr unsigned int HOMEPAGE_LOOP_WAIT_SECS = 10;
} // namespace constants

#endif
