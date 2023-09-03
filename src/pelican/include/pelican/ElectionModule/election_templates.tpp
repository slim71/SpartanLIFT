#ifndef _ELECTION_TEMPLATES_HPP_
#define _ELECTION_TEMPLATES_HPP_

template<typename... Args> void ElectionModule::sendLogDebug(std::string s, Args... args) const {
    this->logger_->logDebug(hb_module, s, args...);
}

template<typename... Args> void ElectionModule::sendLogInfo(std::string s, Args... args) const {
    this->logger_->logInfo(hb_module, s, args...);
}

template<typename... Args> void ElectionModule::sendLogWarning(std::string s, Args... args) const {
    this->logger_->logWarning(hb_module, s, args...);
}

template<typename... Args> void ElectionModule::sendLogError(std::string s, Args... args) const {
    this->logger_->logError(hb_module, s, args...);
}

#endif
