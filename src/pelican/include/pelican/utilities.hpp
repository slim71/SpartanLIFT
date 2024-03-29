#ifndef _UTILITIES_HPP_
#define _UTILITIES_HPP_

#include "types.hpp"

void resetTimer(rclcpp::TimerBase::SharedPtr&);
void cancelTimer(rclcpp::TimerBase::SharedPtr&);
template<typename T> void resetSharedPointer(std::shared_ptr<T>&);
Eigen::Matrix3f rotX(const float, bool = false);
Eigen::Matrix3f rotY(const float, bool = false);
Eigen::Matrix3f rotZ(const float, bool = false);
Eigen::Vector3f quat2RPY(geometry_msgs::msg::Quaternion);

#include "templates.tpp"

#endif
