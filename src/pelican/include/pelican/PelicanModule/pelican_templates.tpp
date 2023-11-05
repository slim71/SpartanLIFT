#ifndef _PELICAN_TEMPLATES_HPP_
#define _PELICAN_TEMPLATES_HPP_

template<typename... Args> void Pelican::sendLogDebug(std::string s, Args... args) const {
    if(this->logger_.isReady())
        this->logger_.logDebug(main_module, s, args...);
    else
        std::cout << "PelicanModule (Missing LoggerModule) [Debug] | " << fmt::format(s, args...) << std::endl;
}

template<typename... Args> void Pelican::sendLogInfo(std::string s, Args... args) const {
    if(this->logger_.isReady())
        this->logger_.logInfo(main_module, s, args...);
    else
        std::cout << "PelicanModule (Missing LoggerModule) [Info] | " << fmt::format(s, args...) << std::endl;
}

template<typename... Args> void Pelican::sendLogWarning(std::string s, Args... args) const {
    if(this->logger_.isReady())
        this->logger_.logWarning(main_module, s, args...);
    else
        std::cout << "PelicanModule (Missing LoggerModule) [Warning] | " << fmt::format(s, args...) << std::endl;
}

template<typename... Args> void Pelican::sendLogError(std::string s, Args... args) const {
    if(this->logger_.isReady())
        this->logger_.logError(main_module, s, args...);
    else
        std::cout << "PelicanModule (Missing LoggerModule) [Error] | " << fmt::format(s, args...) << std::endl;
}

#endif
