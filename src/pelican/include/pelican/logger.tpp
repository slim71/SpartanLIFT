#ifndef _LOGGER_TPP
#define _LOGGER_TPP

template<typename... Args> void LoggerModule::logDebug(possible_modules m, std::string s, Args... args) const {
    RCLCPP_DEBUG_STREAM(
        this->logger_, "[Agent " << this->getID() << "|" << m << "] " << fmt::format(s, args...)
    );
}

template<typename... Args> void LoggerModule::logInfo(possible_modules m, std::string s, Args... args) const {
    RCLCPP_INFO_STREAM(
        this->logger_, "[Agent " << this->getID() << "|" << m << "] " << fmt::format(s, args...)
    );
}

template<typename... Args> void LoggerModule::logWarning(possible_modules m, std::string s, Args... args) const {
    RCLCPP_WARN_STREAM(
        this->logger_, "[Agent " << this->getID() << "|" << m << "] " << fmt::format(s, args...)
    );
}

template<typename... Args> void LoggerModule::logError(possible_modules m, std::string s, Args... args) const {
    RCLCPP_ERROR_STREAM(
        this->logger_, "[Agent " << this->getID() << "|" << m << "] " << fmt::format(s, args...)
    );
}

#endif
