#include "pelican.hpp"

// TODO: check when getters are needed

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
