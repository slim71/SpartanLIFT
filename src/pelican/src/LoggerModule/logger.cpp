#include "LoggerModule/logger.hpp"
#include "PelicanModule/pelican.hpp"

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

void LoggerModule::setID(int i) {
    this->id_ = i;
}

int LoggerModule::getID() const {
    return this->id_;
}

void LoggerModule::initSetup(std::shared_ptr<rclcpp::Logger> logger, int id) {
    this->logger_ = logger;
    this->ready_to_log_ = true;

    this->setID(id);

    this->cached_role_ = tbd;
}

bool LoggerModule::isReady() const {
    return this->ready_to_log_;
}

void LoggerModule::cacheRole(possible_roles role) {
    this->cached_role_ = role;
}
