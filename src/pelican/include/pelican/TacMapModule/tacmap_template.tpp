#ifndef _TACMAP_TEMPLATES_HPP_
#define _TACMAP_TEMPLATES_HPP_

template<typename... Args> void TacMapModule::sendLogDebug(std::string s, Args... args) const {
    if(this->logger_)
        this->logger_->logDebug(tac_module, s, args...);
    else
        std::cout << "TacMapModule | Missing LoggerModule: cannot build proper logs!" << std::endl;
}

template<typename... Args> void TacMapModule::sendLogInfo(std::string s, Args... args) const {
    if(this->logger_)
        this->logger_->logInfo(tac_module, s, args...);
    else
        std::cout << "TacMapModule | Missing LoggerModule: cannot build proper logs!" << std::endl;
}

template<typename... Args> void TacMapModule::sendLogWarning(std::string s, Args... args) const {
    if(this->logger_)
        this->logger_->logWarning(tac_module, s, args...);
    else
        std::cout << "TacMapModule | Missing LoggerModule: cannot build proper logs!" << std::endl;
}

template<typename... Args> void TacMapModule::sendLogError(std::string s, Args... args) const {
    if(this->logger_)
        this->logger_->logError(tac_module, s, args...);
    else
        std::cout << "TacMapModule | Missing LoggerModule: cannot build proper logs!" << std::endl;
}

#endif
