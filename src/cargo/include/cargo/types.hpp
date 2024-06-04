#ifndef _CARGO_TYPES_HPP_
#define _CARGO_TYPES_HPP_

#include "comms/srv/cargo_linkage.hpp"
#include "comms/srv/cargo_point.hpp"
#include "ros_gz_interfaces/srv/set_entity_pose.hpp"
#include <boost/circular_buffer.hpp>
#include <chrono>
#include <fmt/core.h>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <string>

extern geometry_msgs::msg::Point NAN_point;

namespace constants {
    static constexpr unsigned int QOS_HISTORY_AMOUNT = 5;
    static constexpr unsigned int REFERENCE_BUFFER_SIZE = 10;
    static constexpr unsigned int FOLLOW_TIME_MILLISECS = 100;
    static constexpr unsigned int SEARCH_SERVER_STEP_SECS = 1;
    static constexpr unsigned int MAX_SEARCH_TIME_SECS = 10;
    static constexpr unsigned int SERVICE_FUTURE_WAIT_SECS = 1;
} // namespace constants

/*************************** Formatters ****************************/
// Custom formatter specialization to quickly format
// geometry_msgs::msg::Point and nav_msgs::msg::Odometry data with fmt
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

template<> class fmt::formatter<nav_msgs::msg::Odometry> {
    public:
        constexpr auto parse(format_parse_context& ctx) {
            return ctx.begin();
        }

        template<typename Context>
        constexpr auto format(nav_msgs::msg::Odometry const& p, Context& ctx) const {
            return format_to(
                ctx.out(), "({:.4f}, {:.4f}, {:.4f})", p.pose.pose.position.x,
                p.pose.pose.position.y, p.pose.pose.position.z
            );
        }
};

#endif
