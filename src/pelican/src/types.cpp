#include "types.hpp"

#define F(name) #name,
const char* roles_names[] = {ROLES(F) nullptr};
#undef F

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

