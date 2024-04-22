#ifndef _UTILITIES_HPP_
#define _UTILITIES_HPP_

#include "types.hpp"

void resetTimer(rclcpp::TimerBase::SharedPtr&);
void cancelTimer(rclcpp::TimerBase::SharedPtr&);
template<typename T> void resetSharedPointer(std::shared_ptr<T>&);
Eigen::Matrix3d rotX(const double, bool = false);
Eigen::Matrix3d rotY(const double, bool = false);
Eigen::Matrix3d rotZ(const double, bool = false);
Eigen::Vector3d quat2RPY(geometry_msgs::msg::Quaternion);
geometry_msgs::msg::Point
operator-(const geometry_msgs::msg::Point, const geometry_msgs::msg::Point);
Eigen::Vector3d convertENUtoNED(const Eigen::Vector3d&, const Eigen::Vector3d);

#include "templates.tpp"

#endif
