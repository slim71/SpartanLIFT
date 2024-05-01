#ifndef _LOGGER_TPP_
#define _LOGGER_TPP_

template<typename... Args>
void LoggerModule::logDebug(possible_modules m, std::string s, Args... args) const {
    if(this->logger_)
        RCLCPP_DEBUG_STREAM(
            *this->logger_,
            "[Agent " << this->getID() << "|" << m << "|" << this->cached_role_ << "|" << this->cached_term_ << "] " << fmt::format(s, args...)
        );
    else
        std::cout << "Pelican (Missing LoggerModule) [Debug] | " << fmt::format(s, args...) << std::endl;
}

template<typename... Args>
void LoggerModule::logInfo(possible_modules m, std::string s, Args... args) const {
    if(this->logger_)
        RCLCPP_INFO_STREAM(
            *this->logger_,
            "[Agent " << this->getID() << "|" << m << "|" << this->cached_role_ << "|" << this->cached_term_ << "] " << fmt::format(s, args...)
        );
    else
        std::cout << "Pelican (Missing LoggerModule) [Info] | " << fmt::format(s, args...) << std::endl;
}

template<typename... Args>
void LoggerModule::logWarning(possible_modules m, std::string s, Args... args) const {
    if(this->logger_)
        RCLCPP_WARN_STREAM(
            *this->logger_,
            "[Agent " << this->getID() << "|" << m << "|" << this->cached_role_ << "|" << this->cached_term_ << "] " << fmt::format(s, args...)
        );
    else
        std::cout << "Pelican (Missing LoggerModule) [Warning] | " << fmt::format(s, args...) << std::endl;
}

template<typename... Args>
void LoggerModule::logError(possible_modules m, std::string s, Args... args) const {
    if(this->logger_)
        RCLCPP_ERROR_STREAM(
            *this->logger_,
            "[Agent " << this->getID() << "|" << m << "|" << this->cached_role_ << "|" << this->cached_term_ << "] " << fmt::format(s, args...)
        );
    else
        std::cout << "Pelican (Missing LoggerModule) [Error] | " << fmt::format(s, args...) << std::endl;
}

#endif
