/**
 * @file utilities.cpp
 * @author Simone Vollaro (slim71sv@gmail.com)
 * @brief Utility functions used in different part of the package.
 * @version 1.0.0
 * @date 2024-11-14
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "utilities.hpp"

/**
 * @brief Reset a RCLCPP timer.
 *
 * @param timer RCLCPP timer to reset.
 */
void resetTimer(rclcpp::TimerBase::SharedPtr& timer) {
    if (timer) {
        timer->reset();
    }
}

/**
 * @brief Cancel a RCLCPP timer.
 *
 * @param timer RCLCPP timer to cancel.
 */
void cancelTimer(rclcpp::TimerBase::SharedPtr& timer) {
    if (timer) {
        timer->cancel();
    }
}

/**
 * @brief Build a rotation matrix expressing a rotation around the X-axis.
 *
 * @param angle Angle of rotation.
 * @param is_degree Whether the first argument is expressed in degree or not.
 * @return Eigen::Matrix3d
 */
Eigen::Matrix3d rotX(const double angle, bool is_degree) {
    double angle_rad = angle;
    if (is_degree) {
        angle_rad = angle * M_PI / 180.0;
    }

    Eigen::Matrix3d mat {
        {1,              0,               0},
        {0, cos(angle_rad), -sin(angle_rad)},
        {0, sin(angle_rad),  cos(angle_rad)}
    };

    return mat;
}

/**
 * @brief Build a rotation matrix expressing a rotation around the Y-axis.
 *
 * @param angle Angle of rotation.
 * @param is_degree Whether the first argument is expressed in degree or not.
 * @return Eigen::Matrix3d
 */
Eigen::Matrix3d rotY(const double angle, bool is_degree) {
    double angle_rad = angle;
    if (is_degree) {
        angle_rad = angle * M_PI / 180.0;
    }

    Eigen::Matrix3d mat {
        { cos(angle_rad), 0, sin(angle_rad)},
        {              0, 1,              0},
        {-sin(angle_rad), 0, cos(angle_rad)}
    };

    return mat;
}

/**
 * @brief Build a rotation matrix expressing a rotation around the Z-axis.
 *
 * @param angle Angle of rotation.
 * @param is_degree Whether the first argument is expressed in degree or not.
 * @return Eigen::Matrix3d
 */
Eigen::Matrix3d rotZ(const double angle, bool is_degree) {
    double angle_rad = angle;
    if (is_degree) {
        angle_rad = angle * M_PI / 180.0;
    }

    Eigen::Matrix3d mat {
        {cos(angle_rad), -sin(angle_rad), 0},
        {sin(angle_rad),  cos(angle_rad), 0},
        {             0,               0, 1}
    };

    return mat;
}

/**
 * @brief Convert a quaternion to a matrix expressing a rotation in the RPY configuration.
 *
 * @param quat Quaternion to convert.
 * @return Eigen::Vector3d RPY rotation matrix.
 */
Eigen::Vector3d quat2RPY(geometry_msgs::msg::Quaternion quat) {
    double roll, pitch, yaw;
    double r11 = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
    double r21 = 2 * (quat.x * quat.y + quat.z * quat.w);
    double r31 = 2 * (quat.x * quat.z - quat.w * quat.y);
    double r32 = 2 * (quat.w * quat.x + quat.y * quat.z);
    double r33 = 1 - 2 * (quat.x * quat.x + quat.y * quat.y);

    // If the pitch angle is +-90°, there exists a one-parameter family
    // of solutions for the roll and yaw angles: if (roll, 90, yaw) is
    // a solution for a rotation R, then any triple (roll', 90, yaw'), where
    // roll'-yaw'=roll-yaw, is a solution too
    if (abs(r31) == 1) {
        double r12 = 2 * (quat.x * quat.y - quat.z * quat.w);
        double r22 = 1 - 2 * (quat.x * quat.x + quat.z * quat.z);

        pitch = ((r31 < 0) - (r31 > 0)) * 3.14159 / 2;
        roll = 0; // we arbitrarily put roll=0 and compute a yaw angle
        yaw = ((r31 < 0) - (r31 > 0)) * atan2(r12, r22);

    } else {
        pitch = atan2(-r31, sqrt(r11 * r11 + r21 * r21));
        roll = atan2(r32, r33);
        yaw = atan2(r21, r11);
    }

    Eigen::Vector3d result({roll, pitch, yaw});
    return result;
}

/**
 * @brief Convert a vector expressed in the ENU frame to the NED frame.
 *
 * @param enu_pos Vector in ENU frame.
 * @param pos_offset_ENU Offset of the ENU frame.
 * @return Eigen::Vector3d Vector in the NED frame.
 */
Eigen::Vector3d
convertENUtoNED(const Eigen::Vector3d& enu_pos, const Eigen::Vector3d pos_offset_ENU) {
    // Current-axis composition: 1st_rotation * ... * nth_rotation
    Eigen::Matrix3d rot_ENU2NED = rotZ(-90, true) * rotY(180, true);

    // p_NED = - R_ENU2NED * p_ENU
    Eigen::Vector3d pos_offset_NED = -rot_ENU2NED.transpose() * pos_offset_ENU;
    Eigen::Vector4d hom_offset_NED = pos_offset_NED.homogeneous();

    Eigen::Vector4d hom_enu_pos = enu_pos.homogeneous();

    Eigen::Matrix4d hom_ENU2NED;
    hom_ENU2NED.block(0, 0, 3, 3) = rot_ENU2NED;
    hom_ENU2NED.block(3, 0, 1, 3) << 0, 0, 0;
    hom_ENU2NED.col(3) << hom_offset_NED;

    // The yaw rotation is thought to be along the Z_ENU axis
    // Fixed-axis composition: nth_rotation * ... * 1st_rotation
    // Here the first rotation is the one re-aligning the rotated vehicle to the ENU frame
    Eigen::Vector4d body_pos = hom_ENU2NED * hom_enu_pos;

    return body_pos.head(3);
}

/**
 * @brief Operator - redefinition for geometry_msgs::msg::Point objects.
 *
 * @param first
 * @param second
 * @return geometry_msgs::msg::Point Final difference.
 */
geometry_msgs::msg::Point
operator-(const geometry_msgs::msg::Point first, const geometry_msgs::msg::Point second) {
    geometry_msgs::msg::Point diff;
    diff.x = first.x - second.x;
    diff.y = first.y - second.y;
    diff.z = first.z - second.z;

    return diff;
}

/**
 * @brief Operator + redefinition for geometry_msgs::msg::Point objects.
 *
 * @param first
 * @param second
 * @return geometry_msgs::msg::Point Final sum.
 */
geometry_msgs::msg::Point
operator+(const geometry_msgs::msg::Point first, const geometry_msgs::msg::Point second) {
    geometry_msgs::msg::Point sum;
    sum.x = first.x + second.x;
    sum.y = first.y + second.y;
    sum.z = first.z + second.z;

    return sum;
}

/**
 * @brief Compute the position closest to p on the circle with radius circle_radius and centered in
 * center.
 *
 * @param p Position to reference to.
 * @param center Center of the circle.
 * @param circle_radius Radius of the circle.
 * @return geometry_msgs::msg::<Point
 */
geometry_msgs::msg::Point closestCirclePoint(
    geometry_msgs::msg::Point p, geometry_msgs::msg::Point center, double circle_radius
) {
    double dist =
        std::sqrt((p.x - center.x) * (p.x - center.x) + (p.y - center.y) * (p.y - center.y));
    double scale_factor = circle_radius / dist;
    return geometry_msgs::msg::Point()
        .set__x(scale_factor * p.x)
        .set__y(scale_factor * p.y)
        .set__z(p.z);
}

/**
 * @brief Create an array of number positions distributed onto a circle with radius radius and
 * center center.
 *
 * @param start_angle Angle of the first position to generate.
 * @param center Center of the circle.
 * @param radius Radius of the circle.
 * @param number Number of points to generate.
 * @return std::vector<geometry_msgs::msg::Point> Positions generated on the circle.
 */
std::vector<geometry_msgs::msg::Point>
homPointsOnCircle(double start_angle, geometry_msgs::msg::Point center, int radius, int number) {
    std::vector<geometry_msgs::msg::Point> v;
    double angle_increment = 2 * M_PI / number;

    for (int i = 0; i < number; ++i) {
        double angle = start_angle + i * angle_increment;
        double x = center.x + radius * std::cos(angle);
        double y = center.y + radius * std::sin(angle);
        v.push_back(geometry_msgs::msg::Point().set__x(x).set__y(y).set__z(center.z));
    }

    return v;
}

/**
 * @brief Compute the distance of point p from the circle centered in center and with radius
 * circle_radius.
 *
 * @param p Position of which to compute the distance.
 * @param center Center of the circle.
 * @param circle_radius Radius of the circle.
 * @return double
 */
double circleDistance(
    geometry_msgs::msg::Point p, geometry_msgs::msg::Point center, double circle_radius
) {
    return std::abs(
        std::sqrt((p.x - center.x) * (p.x - center.x) + (p.y - center.y) * (p.y - center.y)) -
        circle_radius
    );
}

/**
 * @brief Compute the 2D distance between two points.
 *
 * @param p
 * @param q
 * @return double Distance between p and q.
 */
double p2p2DDistance(geometry_msgs::msg::Point p, geometry_msgs::msg::Point q) {
    return std::sqrt((p.x - q.x) * (p.x - q.x) + (p.y - q.y) * (p.y - q.y));
}

/**
 * @brief Check if the provided point p has any NaN content.
 *
 * @param p
 * @return true
 * @return false
 */
bool geomPointHasNan(geometry_msgs::msg::Point& p) {
    return std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z);
}

/**
 * @brief Create a random double used as direction perturbation.
 *
 * @return double Random number generated.
 */
double random_perturbation() {
    // Generate a random value between -1.0 and 1.0
    double random_value = static_cast<double>(std::rand()) / RAND_MAX * 2.0 - 1.0;

    // Scale the random value to an angle perturbation in radians
    double max_perturbation = M_PI / 32; // Maximum perturbation: 5.625 degrees

    return random_value * max_perturbation;
}

/**
 * @brief Convert a nav_msgs::msg::Odometry object to a geometry_msgs::msg::Point object.
 *
 * @param nav Odometry data expressed in nav_msgs::msg::Odometry.
 * @return geometry_msgs::msg::Point Converted odometry data.
 */
geometry_msgs::msg::Point nav2Geom(nav_msgs::msg::Odometry nav) {
    return geometry_msgs::msg::Point()
        .set__x(nav.pose.pose.position.x)
        .set__y(nav.pose.pose.position.y)
        .set__z(nav.pose.pose.position.z);
}
