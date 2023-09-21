#include "LoggerModule/logger.hpp"
#include "PelicanModule/pelican.hpp"

LoggerModule::LoggerModule() {
    this->logger_ = nullptr;
}

LoggerModule::LoggerModule(std::shared_ptr<rclcpp::Logger> l) {
    if (l)
        this->logger_ = l;
    else
        this->logger_ = nullptr; // CHECK: useless?

    this->ready_to_log_ = true;
}

void LoggerModule::setID(int i) {
    this->id_ = i;
}

int LoggerModule::getID() const {
    return this->id_;
}

void LoggerModule::setupLogger(std::shared_ptr<rclcpp::Logger> l) {
    this->logger_ = l;
    this->ready_to_log_ = true;
}

bool LoggerModule::isReady() const {
    return this->ready_to_log_;
}
