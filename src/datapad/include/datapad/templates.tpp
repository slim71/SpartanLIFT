#ifndef _DATAPAD_TEMPLATES_HPP_
#define _DATAPAD_TEMPLATES_HPP_

template<typename... Args> void Datapad::sendLogDebug(std::string s, Args... args) const {
    if(this->logger_.isReady())
        this->logger_.logDebug(s, args...);
    else
        std::cout << "Datapad (Missing LoggerModule) [Debug] | " << fmt::format(s, args...) << std::endl;
}

template<typename... Args> void Datapad::sendLogInfo(std::string s, Args... args) const {
    if(this->logger_.isReady())
        this->logger_.logInfo(s, args...);
    else
        std::cout << "Datapad (Missing LoggerModule) [Info] | " << fmt::format(s, args...) << std::endl;
}

template<typename... Args> void Datapad::sendLogWarning(std::string s, Args... args) const {
    if(this->logger_.isReady())
        this->logger_.logWarning(s, args...);
    else
        std::cout << "Datapad (Missing LoggerModule) [Warning] | " << fmt::format(s, args...) << std::endl;
}

template<typename... Args> void Datapad::sendLogError(std::string s, Args... args) const {
    if(this->logger_.isReady())
        this->logger_.logError(s, args...);
    else
        std::cout << "Datapad (Missing LoggerModule) [Error] | " << fmt::format(s, args...) << std::endl;
}

#endif
