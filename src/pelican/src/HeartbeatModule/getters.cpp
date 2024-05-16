#include "HeartbeatModule/heartbeat.hpp"

int HeartbeatModule::getNumberOfHbs() const {
    std::lock_guard lock(this->hbs_mutex_);
    return this->received_hbs_.size();
}

heartbeat HeartbeatModule::getLastHb() const {
    std::lock_guard lock(this->hbs_mutex_);
    if (!this->received_hbs_.empty())
        return this->received_hbs_.back();
    else
        return ERROR_HB;
}

int HeartbeatModule::getMaxHbs() const {
    std::lock_guard lock(this->hbs_mutex_);
    return this->received_hbs_.size();
}
