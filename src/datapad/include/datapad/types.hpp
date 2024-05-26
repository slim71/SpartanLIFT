#ifndef _DATAPAD_TYPES_HPP_
#define _DATAPAD_TYPES_HPP_

#include "comms/action/teleop_data.hpp"
#include "comms/srv/cargo_point.hpp"
#include <chrono>
#include <fmt/core.h>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <signal.h>
#include <string>

extern geometry_msgs::msg::Point NAN_point;

enum class TriState { False, True, Floating };

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

/*************************** Formatters ****************************/
// Custom formatter specialization to quickly format
// geometry_msgs::msg::Point and Eigen::Vector3d data with fmt
template<> class fmt::formatter<geometry_msgs::msg::Point> {
    public:
        constexpr auto parse(format_parse_context& ctx) {
            return ctx.begin();
        }

        template<typename Context>
        constexpr auto format(geometry_msgs::msg::Point const& p, Context& ctx) const {
            return format_to(ctx.out(), "({:.4f}, {:.4f}, {:.4f})", p.x, p.y, p.z);
        }
};

#endif
