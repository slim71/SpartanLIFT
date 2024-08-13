#ifndef _PELICAN_TYPES_HPP_
#define _PELICAN_TYPES_HPP_

#include "comms/action/teleop_data.hpp"
#include "comms/msg/command.hpp"
#include "comms/msg/formation_desired.hpp"
#include "comms/msg/heartbeat.hpp"
#include "comms/msg/network_vertex.hpp"
#include "comms/msg/proposal.hpp"
#include "comms/msg/request_vote_rpc.hpp"
#include "comms/srv/cargo_linkage.hpp"
#include "comms/srv/fleet_info.hpp"
#include "comms/srv/formation_reached.hpp"
#include "constants.hpp"
#include "pugixml.hpp"
#include "px4_msgs/msg/failsafe_flags.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/sensor_gps.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_control_mode.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include <Eigen/Geometry>
#include <array>
#include <bitset>
#include <boost/circular_buffer.hpp>
#include <chrono>
#include <cmath>
#include <fmt/core.h>
#include <fmt/format.h>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <math.h>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <queue>
#include <random>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <regex>
#include <signal.h>
#include <std_msgs/msg/empty.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

class MissingExternModule : public std::exception {
    public:
        const char* what() {
            return "No main module present: external functionalities deactivated!";
        }
};

struct vote_count {
        unsigned int candidate_id;
        unsigned int total;
};

struct heartbeat {
        unsigned int term;
        int leader;
        rclcpp::Time timestamp;
};

enum class TriState { False, True, Floating };

std::ostream& operator<<(std::ostream&, const heartbeat&);

extern heartbeat ERROR_HB;
extern geometry_msgs::msg::Point NAN_point;

/********************** Enum Macros / X-Macros *********************/
// Macro "constructors" for type and string tables
#define AS_BARE(a) a,
#define AS_STR(a) #a,

/***************************** Modules *****************************/
// Table; _ for any substitution
#define MODULES(_) \
    _(nullmodule)  \
    _(main_module) \
    _(log_module)  \
    _(hb_module)   \
    _(el_module)   \
    _(tac_module)  \
    _(unsc_module)

enum possible_modules { MODULES(AS_BARE) NumModules };

std::string modules_to_string(possible_modules);
possible_modules string_to_modules(const std::string&);

/****************************** Roles ******************************/
// Table; _ for any substitution
#define ROLES(_) \
    _(tbd)       \
    _(candidate) \
    _(follower)  \
    _(leader)

enum possible_roles { ROLES(AS_BARE) NumPossibleRoles };

std::string roles_to_string(possible_roles);
possible_roles string_to_roles(const std::string&);
std::ostream& operator<<(std::ostream&, const possible_modules&);
std::ostream& operator<<(std::ostream&, const possible_roles&);

/***************************** Commands ****************************/
#define COMMANDS(_)      \
    _(NO_COMMAND)        \
    _(TAKEOFF_COMMAND)   \
    _(LANDING_COMMAND)   \
    _(EMERGENCY_LANDING) \
    _(RENDEZVOUS)        \
    _(FORMATION)

enum supported_commands : uint16_t { COMMANDS(AS_BARE) NumSupportedCommands };

std::string commands_to_string(supported_commands);
std::string commands_to_string(uint16_t);

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

template<> class fmt::formatter<Eigen::Vector3d> {
    public:
        constexpr auto parse(format_parse_context& ctx) {
            return ctx.begin();
        }

        template<typename Context>
        constexpr auto format(Eigen::Vector3d const& v, Context& ctx) const {
            return format_to(ctx.out(), "({}, {}, {})", v[0], v[1], v[2]);
        }
};

#endif
