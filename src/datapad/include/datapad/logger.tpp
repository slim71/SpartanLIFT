#ifndef _LOGGER_TPP_
#define _LOGGER_TPP_

template<typename... Args>
void LoggerModule::logDebug(std::string s, Args... args) const {
    RCLCPP_DEBUG_STREAM(
        *this->logger_, "[Datapad] " << fmt::format(s, args...)
    );
}

template<typename... Args>
void LoggerModule::logInfo(std::string s, Args... args) const {
    RCLCPP_INFO_STREAM(
        *this->logger_, "[Datapad] " << fmt::format(s, args...)
    );
}

template<typename... Args>
void LoggerModule::logWarning(std::string s, Args... args) const {
    RCLCPP_WARN_STREAM(
        *this->logger_, "[Datapad] " << fmt::format(s, args...)
    );
}

template<typename... Args>
void LoggerModule::logError(std::string s, Args... args) const {
    RCLCPP_ERROR_STREAM(
        *this->logger_, "[Datapad] " << fmt::format(s, args...)
    );
}

#endif
