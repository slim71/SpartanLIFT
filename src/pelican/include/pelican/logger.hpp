#ifndef _LOGGER_HPP_
#define _LOGGER_HPP_

#include <string>
#include <rclcpp/logger.hpp>
#include <fmt/core.h>
#include <rclcpp/rclcpp.hpp>

class Pelican;

class LoggerModule {
    public:
        LoggerModule(rclcpp::Logger);
        LoggerModule(rclcpp::Logger, int);

        template<typename... Args> void logInfo(std::string s, Args... args) const;
        template<typename... Args> void logError(std::string s, Args... args) const;
        template<typename... Args> void logWarning(std::string s, Args... args) const;
        template<typename... Args> void logDebug(std::string s, Args... args) const;

        int getID() const;
        void setID(int);

    private:
        rclcpp::Logger logger_;
        int id_;
};

// Including templates definitions
#include "logger.tpp"

#endif