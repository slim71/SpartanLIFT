#ifndef _HEARTBEAT_TEMPLATES_HPP_
#define _HEARTBEAT_TEMPLATES_HPP_

template<typename... Args> void HeartbeatModule::sendLogDebug(std::string s, Args... args) const {
    if(this->logger_)
        this->logger_->logDebug(hb_module, s, args...);
    else
        std::cout << "Missing LoggerModule: cannot build proper logs!" << std::endl;
}

template<typename... Args> void HeartbeatModule::sendLogInfo(std::string s, Args... args) const {
    if(this->logger_)
        this->logger_->logInfo(hb_module, s, args...);
    else
        std::cout << "Missing LoggerModule: cannot build proper logs!" << std::endl;
}

template<typename... Args> void HeartbeatModule::sendLogWarning(std::string s, Args... args) const {
    if(this->logger_)
        this->logger_->logWarning(hb_module, s, args...);
    else
        std::cout << "Missing LoggerModule: cannot build proper logs!" << std::endl;
}

template<typename... Args> void HeartbeatModule::sendLogError(std::string s, Args... args) const {
    if(this->logger_)
        this->logger_->logError(hb_module, s, args...);
    else
        std::cout << "Missing LoggerModule: cannot build proper logs!" << std::endl;
}

#endif
