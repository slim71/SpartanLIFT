#include "types.hpp"

heartbeat ERROR_HB = {0, -1, rclcpp::Time(0, 0)};

const char* modules_names[] = {MODULES(AS_STR) nullptr};
const char* roles_names[] = {ROLES(AS_STR) nullptr};

/**************************** heartbeat ****************************/
std::ostream& operator<<(std::ostream& os, const heartbeat& m) {
    os << "Heartbeat received from leader " << m.leader << " for term " << m.term << " ("
       << m.timestamp.seconds() << "s, " << m.timestamp.nanoseconds() << "ns)";
    return os;
}

/************************* possible_roles **************************/
std::string roles_to_string(possible_roles role) {
    return ((int) role < (int) NumPossibleRoles) ? roles_names[(int) role] : "";
}

possible_roles string_to_roles(const std::string& s) {
    possible_roles r = tbd;
    auto result = std::find(roles_names, roles_names + NumPossibleRoles, s);

    if (result != roles_names + NumPossibleRoles)
        r = possible_roles(result - roles_names);

    return r;
}

std::ostream& operator<<(std::ostream& os, const possible_roles& r) {
    os << roles_to_string(r);
    return os;
}

/************************ possible_modules *************************/
std::string modules_to_string(possible_modules module) {
    return ((int) module < (int) NumModules) ? modules_names[(int) module] : "";
}

possible_modules string_to_modules(const std::string& s) {
    possible_modules m = nullmodule;
    auto result = std::find(modules_names, modules_names + NumModules, s);

    if (result != modules_names + NumModules)
        m = possible_modules(result - modules_names);

    return m;
}

std::ostream& operator<<(std::ostream& os, const possible_modules& m) {
    os << modules_to_string(m);
    return os;
}
