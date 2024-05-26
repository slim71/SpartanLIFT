#ifndef _CARGO_TEMPLATES_HPP_
#define _CARGO_TEMPLATES_HPP_

template<typename... Args> void Cargo::sendLogDebug(std::string s, Args... args) const {
    if(this->logger_.isReady())
        this->logger_.logDebug(s, args...);
    else
        std::cout << "Cargo (Missing LoggerModule) [Debug] | " << fmt::format(s, args...) << std::endl;
}

template<typename... Args> void Cargo::sendLogInfo(std::string s, Args... args) const {
    if(this->logger_.isReady())
        this->logger_.logInfo(s, args...);
    else
        std::cout << "Cargo (Missing LoggerModule) [Info] | " << fmt::format(s, args...) << std::endl;
}

template<typename... Args> void Cargo::sendLogWarning(std::string s, Args... args) const {
    if(this->logger_.isReady())
        this->logger_.logWarning(s, args...);
    else
        std::cout << "Cargo (Missing LoggerModule) [Warning] | " << fmt::format(s, args...) << std::endl;
}

template<typename... Args> void Cargo::sendLogError(std::string s, Args... args) const {
    if(this->logger_.isReady())
        this->logger_.logError(s, args...);
    else
        std::cout << "Cargo (Missing LoggerModule) [Error] | " << fmt::format(s, args...) << std::endl;
}

#endif
