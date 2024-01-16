#ifndef _TYPES_HPP_
#define _TYPES_HPP_

#include "comms/msg/command.hpp"
#include "comms/msg/heartbeat.hpp"
#include "comms/msg/proposal.hpp"
#include "comms/msg/request_vote_rpc.hpp"
#include "comms/msg/status.hpp"
#include "comms/srv/fleet_info_exchange.hpp"
#include "pugixml.hpp"
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
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/transform_datatypes.h>
#include <vector>

// TODO: read
// https://stackoverflow.com/questions/34445336/constexpr-global-constants-in-a-header-file-and-odr
// and move constant to proper source files? maybe with namespaces?
// Used to wait max 30s of missing data from the simulation before stopping
constexpr int MAX_DATA_ATTEMPTS = 30;
constexpr unsigned int MAX_RETRIES = 10;

constexpr unsigned int NO_COMMAND = 0;
constexpr unsigned int TAKEOFF_COMMAND = 1;
constexpr unsigned int LANDING_COMMAND = 2;

class MissingExternModule : public std::exception {
    public:
        const char* what() {
            return "No main module present: external functionalities deactivated!";
        }
};

struct vote_count {
        int candidate_id;
        int total;
};

struct heartbeat {
        unsigned int term;
        int leader;
        rclcpp::Time timestamp;
};

std::ostream& operator<<(std::ostream&, const heartbeat&);

extern heartbeat ERROR_HB;

/***************** Enum Macros / X-Macros ****************/
// Macro "constructors" for type and string tables
#define AS_BARE(a) a,
#define AS_STR(a) #a,

/************************ Modules ************************/
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

/************************* Roles *************************/
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

#endif
