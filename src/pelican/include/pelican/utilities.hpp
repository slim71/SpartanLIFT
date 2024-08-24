#ifndef _UTILITIES_HPP_
#define _UTILITIES_HPP_

#include "types.hpp"

void resetTimer(rclcpp::TimerBase::SharedPtr&);
void cancelTimer(rclcpp::TimerBase::SharedPtr&);
double circleDistance(geometry_msgs::msg::Point, geometry_msgs::msg::Point, double);
double p2p2DDistance(geometry_msgs::msg::Point, geometry_msgs::msg::Point);
bool geomPointHasNan(geometry_msgs::msg::Point&);
Eigen::Matrix3d rotX(const double, bool = false);
Eigen::Matrix3d rotY(const double, bool = false);
Eigen::Matrix3d rotZ(const double, bool = false);
Eigen::Vector3d quat2RPY(geometry_msgs::msg::Quaternion);
Eigen::Vector3d convertENUtoNED(const Eigen::Vector3d&, const Eigen::Vector3d);
geometry_msgs::msg::Point
operator-(const geometry_msgs::msg::Point, const geometry_msgs::msg::Point);
geometry_msgs::msg::Point
operator+(const geometry_msgs::msg::Point, const geometry_msgs::msg::Point);
geometry_msgs::msg::Point
closestCirclePoint(geometry_msgs::msg::Point, geometry_msgs::msg::Point, double);
std::vector<geometry_msgs::msg::Point>
homPointsOnCircle(double, geometry_msgs::msg::Point, int, int);
template<typename T> void resetSharedPointer(std::shared_ptr<T>&);
template<typename T> inline constexpr int signum(T, std::false_type);
template<typename T> inline constexpr int signum(T, std::true_type);
template<typename T> inline constexpr int signum(T);
double random_perturbation();
geometry_msgs::msg::Point nav2Geom(nav_msgs::msg::Odometry);

#include "templates.tpp"

#endif
