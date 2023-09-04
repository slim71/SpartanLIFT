#include "LoggerModule/logger.hpp"

LoggerModule::LoggerModule(rclcpp::Logger l) : logger_(l) {}

LoggerModule::LoggerModule(rclcpp::Logger l, int i) : logger_(l), id_(i) {}

void LoggerModule::setID(int i) {
    this->id_ = i;
}

int LoggerModule::getID() const {
    return this->id_;
}