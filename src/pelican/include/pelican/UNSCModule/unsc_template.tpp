#ifndef _UNSC_TEMPLATES_HPP_
#define _UNSC_TEMPLATES_HPP_

template<typename... Args> void UNSCModule::sendLogDebug(std::string s, Args... args) const {
    if(this->logger_)
        this->logger_->logDebug(unsc_module, s, args...);
    else
        std::cout << "UNSCModule (Missing LoggerModule) [Debug] | " << fmt::format(s, args...) << std::endl;
}

template<typename... Args> void UNSCModule::sendLogInfo(std::string s, Args... args) const {
    if(this->logger_)
        this->logger_->logInfo(unsc_module, s, args...);
    else
        std::cout << "UNSCModule (Missing LoggerModule) [Info] | " << fmt::format(s, args...) << std::endl;
}

template<typename... Args> void UNSCModule::sendLogWarning(std::string s, Args... args) const {
    if(this->logger_)
        this->logger_->logWarning(unsc_module, s, args...);
    else
        std::cout << "UNSCModule (Missing LoggerModule) [Warning] | " << fmt::format(s, args...) << std::endl;
}

template<typename... Args> void UNSCModule::sendLogError(std::string s, Args... args) const {
    if(this->logger_)
        this->logger_->logError(unsc_module, s, args...);
    else
        std::cout << "UNSCModule (Missing LoggerModule) [Error] | " << fmt::format(s, args...) << std::endl;
}

#endif
