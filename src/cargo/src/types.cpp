/**
 * @file types.cpp
 * @author Simone Vollaro (slim71sv@gmail.com)
 * @brief Common structures used in the Cargo package.
 * @version 1.0.0
 * @date 2024-11-13
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "types.hpp"

/************************** NAN structure **************************/
/**
 * @brief Object with NaN values, used as standard content.
 */
geometry_msgs::msg::Point NAN_point =
    geometry_msgs::msg::Point().set__x(std::nan("")).set__y(std::nan("")).set__z(std::nan(""));
