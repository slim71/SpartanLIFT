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
