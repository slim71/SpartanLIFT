/**
 * @file types.cpp
 * @author Simone Vollaro (slim71sv@gmail.com)
 * @brief File containing anything related to custom types defined.
 * @version 1.0.0
 * @date 2024-11-13
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "types.hpp"
#include "utilities.hpp"

heartbeat ERROR_HB = {0, -1, rclcpp::Time(0, 0)};

const char* modules_names[] = {MODULES(AS_STR) nullptr};
const char* roles_names[] = {ROLES(AS_STR) nullptr};
const char* commands_names[] = {COMMANDS(AS_STR) nullptr};

/**************************** heartbeat ****************************/
/**
 * @brief Redefinition of the << operator to format heartbeat info.
 *
 * @param os Output stream.
 * @param m Heartbeat object to format.
 * @return std::ostream&
 */
std::ostream& operator<<(std::ostream& os, const heartbeat& m) {
    os << "Heartbeat received from leader " << m.leader << " for term " << m.term << " ("
       << m.timestamp.seconds() << "s, " << m.timestamp.nanoseconds() << "ns)";
    return os;
}

/************************* possible_roles **************************/
/**
 * @brief Converter to string for the possible_roles structure.
 *
 * @param role possible_roles object to convert.
 */
std::string roles_to_string(possible_roles role) {
    return ((int) role < (int) NumPossibleRoles) ? roles_names[(int) role] : "";
}

/**
 * @brief Converter to possible_roles for a string.
 *
 * @param s String to convert.
 */
possible_roles string_to_roles(const std::string& s) {
    possible_roles r = tbd;
    auto result = std::find(roles_names, roles_names + NumPossibleRoles, s);

    if (result != roles_names + NumPossibleRoles)
        r = possible_roles(result - roles_names);

    return r;
}

/**
 * @brief Redefinition of the << operator to print possible_roles objects as string.
 *
 * @param os Output stream.
 * @param r possible_roles object to print.
 * @return std::ostream&
 */
std::ostream& operator<<(std::ostream& os, const possible_roles& r) {
    os << roles_to_string(r);
    return os;
}

/************************ possible_modules *************************/
/**
 * @brief Converter to string for the possible_modules structure.
 *
 * @param role possible_modules object to convert.
 */
std::string modules_to_string(possible_modules module) {
    return ((int) module < (int) NumModules) ? modules_names[(int) module] : "";
}

/**
 * @brief Converter to possible_modules for a string.
 *
 * @param s String to convert.
 */
possible_modules string_to_modules(const std::string& s) {
    possible_modules m = nullmodule;
    auto result = std::find(modules_names, modules_names + NumModules, s);

    if (result != modules_names + NumModules)
        m = possible_modules(result - modules_names);

    return m;
}

/**
 * @brief Redefinition of the << operator to print possible_modules objects as string.
 *
 * @param os Output stream.
 * @param m possible_modules object to print.
 * @return std::ostream&
 */
std::ostream& operator<<(std::ostream& os, const possible_modules& m) {
    os << modules_to_string(m);
    return os;
}

/*********************** supported_commands ************************/
/**
 * @brief Converter to string for the supported_commands structure.
 *
 * @param command supported_commands object to convert.
 */
std::string commands_to_string(supported_commands command) {
    return ((uint16_t) command < (uint16_t) NumSupportedCommands)
               ? commands_names[(uint16_t) command]
               : "";
}

/**
 * @brief Converter to string for a uint16_t.
 *
 * @param command uint16_t to convert.
 */
std::string commands_to_string(uint16_t command) {
    return (command < (uint16_t) NumSupportedCommands) ? commands_names[command] : "";
}

/************************** NAN structure **************************/
/**
 * @brief Object with NaN values, used as standard content.
 */
geometry_msgs::msg::Point NAN_point =
    geometry_msgs::msg::Point().set__x(std::nan("")).set__y(std::nan("")).set__z(std::nan(""));
