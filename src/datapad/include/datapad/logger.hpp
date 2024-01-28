#ifndef _LOGGER_HPP_
#define _LOGGER_HPP_

#include "types.hpp"

class LoggerModule {
    public:
        explicit LoggerModule();
        explicit LoggerModule(std::shared_ptr<rclcpp::Logger>);

        template<typename... Args> void logInfo(std::string, Args...) const;
        template<typename... Args> void logError(std::string, Args...) const;
        template<typename... Args> void logWarning(std::string, Args...) const;
        template<typename... Args> void logDebug(std::string, Args...) const;

        void initSetup(std::shared_ptr<rclcpp::Logger>);

        bool isReady() const;

    private:
        std::shared_ptr<rclcpp::Logger> logger_;
        bool ready_to_log_ {false};
};

// Including templates definitions
#include "logger.tpp"

#endif
