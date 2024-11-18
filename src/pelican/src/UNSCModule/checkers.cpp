/**
 * @file checkers.cpp
 * @author Simone Vollaro (slim71sv@gmail.com)
 * @brief Methods checking core conditions and handling related promises.
 * @version 1.0.0
 * @date 2024-11-14
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "PelicanModule/pelican.hpp"
#include "UNSCModule/unsc.hpp"

/**
 * @brief Check if the formation has been achieved and fulfill the related promise.
 *
 */
void UNSCModule::checkFormationAchieved() {
    if (this->confirmFormationAchieved()) {
        this->sendLogDebug("Formation achieved!");

        cancelTimer(this->formation_check_timer_);
        this->fa_promise_.set_value(); // Fulfill the promise
    } else {
        this->sendLogDebug("Formation not yet achieved");
    }
}

/**
 * @brief Check if the reference height has been achieved and fulfill the related promise.
 *
 */
void UNSCModule::checkHeightReached() {
    if (abs(this->getActualTargetHeight() - this->gatherCopterPosition(this->gatherAgentID()).z) <
        0.1) {
        this->sendLogDebug("Height reached!");

        cancelTimer(this->height_check_timer_);
        this->fa_promise_.set_value(); // Fulfill the promise
    } else {
        this->sendLogDebug("Formation not yet achieved");
    }
}

/**
 * @brief Check if the desired target has been achieved and fulfill the related promise.
 *
 */
void UNSCModule::checkTargetReached() {
    std::optional<geometry_msgs::msg::Point> maybe_target = this->getTargetPosition();
    if (!maybe_target) {
        this->sendLogWarning("Target data not available!");
        return;
    }
    geometry_msgs::msg::Point target_pos = maybe_target.value();
    double target_height = this->getActualTargetHeight();

    this->sendLogDebug("Checking if target position has been reached");
    geometry_msgs::msg::Point pos = this->gatherCopterPosition(this->gatherAgentID());
    this->sendLogDebug("Target: {}, last recorded: {}", target_pos, pos);

    if ((abs(target_pos.x - pos.x) <= constants::SETPOINT_REACHED_DISTANCE) &&
        (abs(target_pos.y - pos.y) <= constants::SETPOINT_REACHED_DISTANCE) &&
        (abs(target_height - pos.z) <= constants::SETPOINT_REACHED_DISTANCE)) {
        this->sendLogDebug("Target reached!");
        cancelTimer(this->target_check_timer_);
        this->fa_promise_.set_value();
    } else {
        geometry_msgs::msg::Point vel =
            geometry_msgs::msg::Point()
                .set__x(
                    (target_pos.x - pos.x) /
                    (constants::DELAY_MILLIS / constants::MILLIS_TO_SECS_CONVERSION)
                )
                .set__y(
                    (target_pos.y - pos.y) /
                    (constants::DELAY_MILLIS / constants::MILLIS_TO_SECS_CONVERSION)
                )
                .set__z(std::nanf("")); // Do not handle the z axis (height)
        this->sendLogDebug("Computed next velocity setpoint for pre-formation operations: {}", vel);
    }
}
