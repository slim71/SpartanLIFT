#ifndef _PELICAN_LOGGER_TPP
#define _PELICAN_LOGGER_TPP

template<typename... Args> void Pelican::logInfo(std::string s, Args... args) const {
    RCLCPP_INFO_STREAM(
        this->get_logger(), "[Agent " << this->getID() << "] " << fmt::format(s, args...)
    );
}

template<typename... Args> void Pelican::logDebug(std::string s, Args... args) const {
    RCLCPP_DEBUG_STREAM(
        this->get_logger(), "[Agent " << this->getID() << "] " << fmt::format(s, args...)
    );
}

template<typename... Args> void Pelican::logError(std::string s, Args... args) const {
    RCLCPP_ERROR_STREAM(
        this->get_logger(), "[Agent " << this->getID() << "] " << fmt::format(s, args...)
    );
}

template<typename... Args> void Pelican::logWarning(std::string s, Args... args) const {
    RCLCPP_WARN_STREAM(
        this->get_logger(), "[Agent " << this->getID() << "] " << fmt::format(s, args...)
    );
}

#endif
