#ifndef _LOGGER_TPP
#define _LOGGER_TPP

template<typename... Args> void LoggerModule::logInfo(std::string s, possible_modules m, Args... args) const {
    RCLCPP_INFO_STREAM(
        this->logger_, "[Agent " << this->getID() << "|" << m << "] " << fmt::format(s, args...)
    );
}

template<typename... Args> void LoggerModule::logDebug(std::string s, possible_modules m, Args... args) const {
    RCLCPP_DEBUG_STREAM(
        this->logger_, "[Agent " << this->getID() << "|" << m << "] " << fmt::format(s, args...)
    );
}

template<typename... Args> void LoggerModule::logError(std::string s, possible_modules m, Args... args) const {
    RCLCPP_ERROR_STREAM(
        this->logger_, "[Agent " << this->getID() << "|" << m << "] " << fmt::format(s, args...)
    );
}

template<typename... Args> void LoggerModule::logWarning(std::string s, possible_modules m, Args... args) const {
    RCLCPP_WARN_STREAM(
        this->logger_, "[Agent " << this->getID() << "|" << m << "] " << fmt::format(s, args...)
    );
}

#endif
