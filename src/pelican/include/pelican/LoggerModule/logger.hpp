#ifndef _LOGGER_HPP_
#define _LOGGER_HPP_

#include "types.hpp"
#include <fmt/core.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

class Pelican;

class LoggerModule {
    public:
        explicit LoggerModule();
        explicit LoggerModule(std::shared_ptr<rclcpp::Logger>);

        template<typename... Args> void logInfo(possible_modules, std::string, Args...) const;
        template<typename... Args> void logError(possible_modules, std::string, Args...) const;
        template<typename... Args> void logWarning(possible_modules, std::string, Args...) const;
        template<typename... Args> void logDebug(possible_modules, std::string, Args...) const;

        int getID() const;
        void setID(int);

        void setupLogger(std::shared_ptr<rclcpp::Logger>);

        bool isReady() const;

    private:
        std::shared_ptr<rclcpp::Logger> logger_;
        int id_ {0};
        bool ready_to_log_ {false};
};

// Including templates definitions
#include "logger.tpp"

#endif
