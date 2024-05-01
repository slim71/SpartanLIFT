#ifndef _LOGGER_TPP_
#define _LOGGER_TPP_

template<typename... Args>
void LoggerModule::logDebug(std::string s, Args... args) const {
    if(this->logger_)
        RCLCPP_DEBUG_STREAM(
            *this->logger_, "[Datapad] " << fmt::format(s, args...)
        );
    else
        std::cout << "Datapad (Missing LoggerModule) [Debug] | " << fmt::format(s, args...) << std::endl;
}

template<typename... Args>
void LoggerModule::logInfo(std::string s, Args... args) const {
    if(this->logger_)
        RCLCPP_INFO_STREAM(
            *this->logger_, "[Datapad] " << fmt::format(s, args...)
        );
    else
        std::cout << "Datapad (Missing LoggerModule) [Info] | " << fmt::format(s, args...) << std::endl;
}

template<typename... Args>
void LoggerModule::logWarning(std::string s, Args... args) const {
    if(this->logger_)
        RCLCPP_WARN_STREAM(
            *this->logger_, "[Datapad] " << fmt::format(s, args...)
        );
    else
        std::cout << "Datapad (Missing LoggerModule) [Warning] | " << fmt::format(s, args...) << std::endl;
}

template<typename... Args>
void LoggerModule::logError(std::string s, Args... args) const {
    if(this->logger_)
        RCLCPP_ERROR_STREAM(
            *this->logger_, "[Datapad] " << fmt::format(s, args...)
        );
    else
        std::cout << "Datapad (Missing LoggerModule) [Error] | " << fmt::format(s, args...) << std::endl;
}

#endif
