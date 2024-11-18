/**
 * @file logger.cpp
 * @author Simone Vollaro (slim71sv@gmail.com)
 * @brief Fiel containing methods of the LoggerModule class.
 * @version 1.0.0
 * @date 2024-11-13
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "logger.hpp"

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
 * @brief Initialize the LoggerModule object.
 *
 * @param logger RCLCPP logger to link.
 */
void LoggerModule::initSetup(std::shared_ptr<rclcpp::Logger> logger) {
    this->logger_ = logger;
    this->ready_to_log_ = true;
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
