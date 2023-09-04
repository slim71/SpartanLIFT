#include "HeartbeatModule/heartbeat.hpp"

int HeartbeatModule::getNumberOfHbs() const {
    std::lock_guard<std::mutex> lock(this->hbs_mutex_);
    return this->received_hbs_.size();
}

heartbeat HeartbeatModule::getLastHb() const {
    std::lock_guard<std::mutex> lock(this->hbs_mutex_);
    return this->received_hbs_.back();
}

int HeartbeatModule::getMaxHbs() const {
    std::lock_guard<std::mutex> lock(this->hbs_mutex_);
    return this->received_hbs_.size();
}
