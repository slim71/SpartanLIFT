#ifndef _PELICAN_TEMPLATES_HPP_
#define _PELICAN_TEMPLATES_HPP_

template<typename... Args> void Pelican::sendLogDebug(std::string s, Args... args) const {
    this->logger_.logDebug(main_module, s, args...);
}

template<typename... Args> void Pelican::sendLogInfo(std::string s, Args... args) const {
    this->logger_.logInfo(main_module, s, args...);
}

template<typename... Args> void Pelican::sendLogWarning(std::string s, Args... args) const {
    this->logger_.logWarning(main_module, s, args...);
}

template<typename... Args> void Pelican::sendLogError(std::string s, Args... args) const {
    this->logger_.logError(main_module, s, args...);
}

#endif
