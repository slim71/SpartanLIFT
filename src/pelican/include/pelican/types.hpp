#include <rclcpp/rclcpp.hpp>

struct vote_count {
        int candidate_id;
        int total;
};

struct heartbeat {
        int term;
        int leader;
        rclcpp::Time timestamp;
};

#define ROLES(F) \
    F(tbd)       \
    F(candidate) \
    F(follower)  \
    F(leader)

#define F(name) name,
enum possible_roles { ROLES(F) NumPossibleRoles };
#undef F

std::string roles_to_string(possible_roles);
possible_roles string_to_roles(const std::string&);
