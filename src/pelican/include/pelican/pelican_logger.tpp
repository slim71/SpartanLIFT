#ifndef _PELICAN_LOGGER_TPP
#define _PELICAN_LOGGER_TPP

// TODO: move externally?

template <typename... Args>
void PelicanUnit::logInfo(std::string s, Args... args) {
    RCLCPP_INFO_STREAM(this->get_logger(), "[Agent " << this->getID() << "] " << fmt::format(s, args...));
}

template <typename... Args>
void PelicanUnit::logDebug(std::string s, Args... args) {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "[Agent " << this->getID() << "] " << fmt::format(s, args...));
}

template <typename... Args>
void PelicanUnit::logError(std::string s, Args... args) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "[Agent " << this->getID() << "] " << fmt::format(s, args...));
}

template <typename... Args>
void PelicanUnit::logWarning(std::string s, Args... args) {
    RCLCPP_WARN_STREAM(this->get_logger(), "[Agent " << this->getID() << "] " << fmt::format(s, args...));
}

#endif