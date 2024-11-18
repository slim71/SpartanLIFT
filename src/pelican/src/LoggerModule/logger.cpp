/**
 * @file logger.cpp
 * @author Simone Vollaro (slim71sv@gmail.com)
 * @brief File containing methods of the LoggerModule class.
 * @version 1.0.0
 * @date 2024-11-13
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "LoggerModule/logger.hpp"
#include "PelicanModule/pelican.hpp"

/**
 * @brief Construct a new LoggerModule object.
 *
 */
LoggerModule::LoggerModule() {
    this->logger_ = nullptr;
}

/**
 * @brief Construct a new LoggerModule object.
 *
 * @param logger RCLCPP logger to link.
 */
LoggerModule::LoggerModule(std::shared_ptr<rclcpp::Logger> logger) {
    if (logger) {
        this->logger_ = logger;
        this->ready_to_log_ = true;
    } else
        this->logger_ = nullptr;
}

/**
 * @brief Destroy the LoggerModule object.
 *
 */
LoggerModule::~LoggerModule() {
    // Ensures the logger shared pointer is properly released.
    logger_.reset();
}

/**
 * @brief Set the internal reference to the linked agent.
 *
 * @param i ID of the linked agent.
 */
void LoggerModule::setID(int i) {
    this->id_ = i;
}

/**
 * @brief Get the ID of the linked agent.
 *
 * @return int ID of the linked agent.
 */
int LoggerModule::getID() const {
    return this->id_;
}

/**
 * @brief Initialize the LoggerModule object.
 *
 * @param logger RCLCPP logger to link.
 */
void LoggerModule::initSetup(std::shared_ptr<rclcpp::Logger> logger, int id) {
    this->logger_ = logger;
    this->ready_to_log_ = true;

    this->setID(id);
}

/**
 * @brief Return the status of the ready_to_log_ flag.
 *
 * @return true
 * @return false
 */
bool LoggerModule::isReady() const {
    return this->ready_to_log_;
}

/**
 * @brief Cache the role of the linked agent internally.
 *
 * @param role Role of the linked agent.
 */
void LoggerModule::cacheRole(possible_roles role) {
    this->cached_role_ = role;
}

/**
 * @brief Cache the term of the linked agent internally.
 *
 * @param term Term of the linked agent.
 */
void LoggerModule::cacheTerm(unsigned int term) {
    this->cached_term_ = term;
}
