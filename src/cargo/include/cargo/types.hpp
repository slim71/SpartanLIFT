#ifndef _CARGO_TYPES_HPP_
#define _CARGO_TYPES_HPP_

#include "comms/srv/cargo_point.hpp"
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
