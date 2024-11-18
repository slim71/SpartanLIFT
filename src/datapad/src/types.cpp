/**
 * @file types.cpp
 * @author Simone Vollaro (slim71sv@gmail.com)
 * @brief File containing anything related to custom types defined.
 * @version 1.0.0
 * @date 2024-11-13
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "types.hpp"

/**************************** Aggregates ***************************/
/**
 * @brief Default constructor for the Flags object.
 */
Flags::Flags() {};

/**
 * @brief Set the presence flag of the Flags object.
 *
 * @return Flags& Same object that has been modified.
 */
Flags& Flags::SetPresence() {
    this->presence = true;
    return *this;
}

/**
 * @brief Set the takeoff flag of the Flags object.
 *
 * @return Flags& Same object that has been modified.
 */
Flags& Flags::SetTakeoff() {
    this->takeoff = true;
    return *this;
}

/**
 * @brief Set the landing flag of the Flags object.
 *
 * @return Flags& Same object that has been modified.
 */
Flags& Flags::SetLanding() {
    this->landing = true;
    return *this;
}

/**
 * @brief Set the retrieval flag of the Flags object.
 *
 * @return Flags& Same object that has been modified.
 */
Flags& Flags::SetRetrieval() {
    this->retrieval = true;
    return *this;
}

/**
 * @brief Set the dropoff flag of the Flags object.
 *
 * @return Flags& Same object that has been modified.
 */
Flags& Flags::SetDropoff() {
    this->dropoff = true;
    return *this;
}

/**
 * @brief Get the status of the presence flag.
 *
 * @return true
 * @return false
 */
bool Flags::GetPresence() const {
    return this->presence;
}

/**
 * @brief Get the status of the takeoff flag.
 *
 * @return true
 * @return false
 */
bool Flags::GetTakeoff() const {
    return this->takeoff;
}

/**
 * @brief Get the status of the landing flag.
 *
 * @return true
 * @return false
 */
bool Flags::GetLanding() const {
    return this->landing;
}

/**
 * @brief Get the status of the retrieval flag.
 *
 * @return true
 * @return false
 */
bool Flags::GetRetrieval() const {
    return this->retrieval;
}

/**
 * @brief Get the status of the dropoff flag.
 *
 * @return true
 * @return false
 */
bool Flags::GetDropoff() const {
    return this->dropoff;
}

/************************** NAN structure **************************/
/**
 * @brief Object with NaN values, used as standard content.
 */
geometry_msgs::msg::Point NAN_point =
    geometry_msgs::msg::Point().set__x(std::nan("")).set__y(std::nan("")).set__z(std::nan(""));
