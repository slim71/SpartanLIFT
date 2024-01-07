#include "logger.hpp"

LoggerModule::LoggerModule() {
    this->logger_ = nullptr;
}

LoggerModule::LoggerModule(std::shared_ptr<rclcpp::Logger> logger) {
    if (logger) {
        this->logger_ = logger;
        this->ready_to_log_ = true;
    } else
        this->logger_ = nullptr;
}

void LoggerModule::initSetup(std::shared_ptr<rclcpp::Logger> logger) {
    this->logger_ = logger;
    this->ready_to_log_ = true;
}

bool LoggerModule::isReady() const {
    return this->ready_to_log_;
}
