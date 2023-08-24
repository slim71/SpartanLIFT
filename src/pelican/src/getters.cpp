#include "pelican.hpp"

int Pelican::getID() const {
    return this->id_;
}

std::string Pelican::getModel() const {
    return this->model_;
}

double Pelican::getMass() const {
    return this->mass_;
}

possible_roles Pelican::getRole() const {
    return this->role_;
}

int Pelican::getCurrentTerm() const {
    return this->current_term_;
}

std::shared_ptr<Pelican> Pelican::getInstance() {
    return instance_.lock();
}

std::chrono::milliseconds Pelican::getBallotWaitTime() const {
    return this->new_ballot_waittime_;
}

int Pelican::getNumberOfHbs() const {
    std::lock_guard<std::mutex> lock(this->hbs_mutex_);
    return this->received_hbs_.size();
}

heartbeat Pelican::getLastHb() const {
    std::lock_guard<std::mutex> lock(this->hbs_mutex_);
    return this->received_hbs_.back();
}

int Pelican::getMaxHbs() const {
    std::lock_guard<std::mutex> lock(this->hbs_mutex_);
    return this->received_hbs_.size();
}
