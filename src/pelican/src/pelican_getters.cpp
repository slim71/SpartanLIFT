#include "pelican.hpp"

int PelicanUnit::getID() const { 
    return this->id_; 
}

std::string PelicanUnit::getName() const {
    return this->name_;
}

std::string PelicanUnit::getModel() const {
    return this->model_;
}

double PelicanUnit::getMass() const {
    return this->mass_;
}

possible_roles PelicanUnit::getRole() const {
    return this->role_;
}

int PelicanUnit::getCurrentTerm() const {
    return this->current_term_;
}

std::shared_ptr<PelicanUnit> PelicanUnit::getInstance() {
    return instance_.lock();
}

std::chrono::milliseconds PelicanUnit::getBallotWaitTime() {
    return this->new_ballot_waittime_;
}