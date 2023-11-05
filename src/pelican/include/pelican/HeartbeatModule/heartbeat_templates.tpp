#ifndef _HEARTBEAT_TEMPLATES_HPP_
#define _HEARTBEAT_TEMPLATES_HPP_

template<typename... Args> void HeartbeatModule::sendLogDebug(std::string s, Args... args) const {
    if(this->logger_)
        this->logger_->logDebug(hb_module, s, args...);
    else
        std::cout << "HeartbeatModule (Missing LoggerModule) [Debug] | " << fmt::format(s, args...) << std::endl;
}

template<typename... Args> void HeartbeatModule::sendLogInfo(std::string s, Args... args) const {
    if(this->logger_)
        this->logger_->logInfo(hb_module, s, args...);
    else
        std::cout << "HeartbeatModule (Missing LoggerModule) [Info] | " << fmt::format(s, args...) << std::endl;
}

template<typename... Args> void HeartbeatModule::sendLogWarning(std::string s, Args... args) const {
    if(this->logger_)
        this->logger_->logWarning(hb_module, s, args...);
    else
        std::cout << "HeartbeatModule (Missing LoggerModule) [Warning] | " << fmt::format(s, args...) << std::endl;
}

template<typename... Args> void HeartbeatModule::sendLogError(std::string s, Args... args) const {
    if(this->logger_)
        this->logger_->logError(hb_module, s, args...);
    else
        std::cout << "HeartbeatModule (Missing LoggerModule) [Error] | " << fmt::format(s, args...) << std::endl;
}

#endif
