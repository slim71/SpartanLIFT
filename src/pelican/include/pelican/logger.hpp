#ifndef _LOGGER_HPP_
#define _LOGGER_HPP_

#include <string>
#include <rclcpp/logger.hpp>
#include <fmt/core.h>
#include <rclcpp/rclcpp.hpp>
#include "types.hpp"

class Pelican;

class LoggerModule {
    public:
        explicit LoggerModule(rclcpp::Logger);
        explicit LoggerModule(rclcpp::Logger, int);

        template<typename... Args> void logInfo(possible_modules, std::string, Args...) const;
        template<typename... Args> void logError(possible_modules, std::string, Args...) const;
        template<typename... Args> void logWarning(possible_modules, std::string, Args...) const;
        template<typename... Args> void logDebug(possible_modules, std::string, Args...) const;

        int getID() const;
        void setID(int);

    private:
        rclcpp::Logger logger_;
        int id_;
};

// Including templates definitions
#include "logger.tpp"

#endif