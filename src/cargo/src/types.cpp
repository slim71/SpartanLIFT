#include "types.hpp"

/************************** NAN structure **************************/
geometry_msgs::msg::Point NAN_point =
    geometry_msgs::msg::Point().set__x(std::nan("")).set__y(std::nan("")).set__z(std::nan(""));
