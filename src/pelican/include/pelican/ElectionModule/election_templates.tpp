#ifndef _ELECTION_TEMPLATES_HPP_
#define _ELECTION_TEMPLATES_HPP_

template<typename... Args> void ElectionModule::sendLogDebug(std::string s, Args... args) const {
    if(this->logger_)
        this->logger_->logDebug(el_module, s, args...);
    else
        std::cout << "ElectionModule (Missing LoggerModule) [Debug] | " << fmt::format(s, args...) << std::endl;
}

template<typename... Args> void ElectionModule::sendLogInfo(std::string s, Args... args) const {
    if(this->logger_)
        this->logger_->logInfo(el_module, s, args...);
    else
        std::cout << "ElectionModule (Missing LoggerModule) [Info] | " << fmt::format(s, args...) << std::endl;
}

template<typename... Args> void ElectionModule::sendLogWarning(std::string s, Args... args) const {
    if(this->logger_)
        this->logger_->logWarning(el_module, s, args...);
    else
        std::cout << "ElectionModule (Missing LoggerModule) [Warning] | " << fmt::format(s, args...) << std::endl;
}

template<typename... Args> void ElectionModule::sendLogError(std::string s, Args... args) const {
    if(this->logger_)
        this->logger_->logError(el_module, s, args...);
    else
        std::cout << "ElectionModule (Missing LoggerModule) [Error] | " << fmt::format(s, args...) << std::endl;
}

#endif
