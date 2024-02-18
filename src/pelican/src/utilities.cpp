#include "utilities.hpp"

void resetTimer(rclcpp::TimerBase::SharedPtr& timer) {
    if (timer) {
        timer->reset();
    }
}

void cancelTimer(rclcpp::TimerBase::SharedPtr& timer) {
    if (timer) {
        timer->cancel();
    }
}

Eigen::Matrix3f rotX(const float angle, bool is_degree) {
    float angle_rad = angle;
    if (is_degree) {
        angle_rad = angle * constants::PI / 180.0;
    }

    Eigen::Matrix3f mat {
        {1,              0,               0},
        {0, cos(angle_rad), -sin(angle_rad)},
        {0, sin(angle_rad),  cos(angle_rad)}
    };

    return mat;
}

Eigen::Matrix3f rotY(const float angle, bool is_degree) {
    float angle_rad = angle;
    if (is_degree) {
        angle_rad = angle * constants::PI / 180.0;
    }

    Eigen::Matrix3f mat {
        { cos(angle_rad), 0, sin(angle_rad)},
        {              0, 1,              0},
        {-sin(angle_rad), 0, cos(angle_rad)}
    };

    return mat;
}

Eigen::Matrix3f rotZ(const float angle, bool is_degree) {
    float angle_rad = angle;
    if (is_degree) {
        angle_rad = angle * constants::PI / 180.0;
    }

    Eigen::Matrix3f mat {
        {cos(angle_rad), -sin(angle_rad), 0},
        {sin(angle_rad),  cos(angle_rad), 0},
        {             0,               0, 1}
    };

    return mat;
}

Eigen::Vector3f quat2RPY(geometry_msgs::msg::Quaternion quat) {
    float roll, pitch, yaw;
    float r11 = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
    float r21 = 2 * (quat.x * quat.y + quat.z * quat.w);
    float r31 = 2 * (quat.x * quat.z - quat.w * quat.y);
    float r32 = 2 * (quat.w * quat.x + quat.y * quat.z);
    float r33 = 1 - 2 * (quat.x * quat.x + quat.y * quat.y);

    // If the pitch angle is +-90°, there exists a one-parameter family
    // of solutions for the roll and yaw angles: if (roll, 90, yaw) is
    // a solution for a rotation R, then any triple (roll', 90, yaw'), where
    // roll'-yaw'=roll-yaw, is a solution too
    if (abs(r31) == 1) {
        float r12 = 2 * (quat.x * quat.y - quat.z * quat.w);
        float r22 = 1 - 2 * (quat.x * quat.x + quat.z * quat.z);

        pitch = ((r31 < 0) - (r31 > 0)) * 3.14159 / 2;
        roll = 0; // we arbitrarily put roll=0 and compute a yaw angle
        yaw = ((r31 < 0) - (r31 > 0)) * atan2(r12, r22);

    } else {
        pitch = atan2(-r31, sqrt(r11 * r11 + r21 * r21));
        roll = atan2(r32, r33);
        yaw = atan2(r21, r11);
    }

    Eigen::Vector3f result({roll, pitch, yaw});
    return result;
}
