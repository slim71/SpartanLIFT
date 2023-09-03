#ifndef _TYPES_HPP_
#define _TYPES_HPP_

#include <rclcpp/rclcpp.hpp>

#define EXTERNAL_OFF "No main module present: external functionalities deactivated!"

struct vote_count {
        int candidate_id;
        int total;
};

struct heartbeat {
        int term;
        int leader;
        rclcpp::Time timestamp;
};

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
    _(hb_module)

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
