#ifndef _TACMAP_TEMPLATES_HPP_
#define _TACMAP_TEMPLATES_HPP_

template<typename... Args> void TacMapModule::sendLogDebug(std::string s, Args... args) const {
    if(this->logger_)
        this->logger_->logDebug(tac_module, s, args...);
    else
        std::cout << "TacMapModule (Missing LoggerModule) [Debug] | " << fmt::format(s, args...) << std::endl;
}

template<typename... Args> void TacMapModule::sendLogInfo(std::string s, Args... args) const {
    if(this->logger_)
        this->logger_->logInfo(tac_module, s, args...);
    else
        std::cout << "TacMapModule (Missing LoggerModule) [Info] | " << fmt::format(s, args...) << std::endl;
}

template<typename... Args> void TacMapModule::sendLogWarning(std::string s, Args... args) const {
    if(this->logger_)
        this->logger_->logWarning(tac_module, s, args...);
    else
        std::cout << "TacMapModule (Missing LoggerModule) [Warning] | " << fmt::format(s, args...) << std::endl;
}

template<typename... Args> void TacMapModule::sendLogError(std::string s, Args... args) const {
    if(this->logger_)
        this->logger_->logError(tac_module, s, args...);
    else
        std::cout << "TacMapModule (Missing LoggerModule) [Error] | " << fmt::format(s, args...) << std::endl;
}

#endif
