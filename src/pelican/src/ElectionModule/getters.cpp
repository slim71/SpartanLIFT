#include "ElectionModule/election.hpp"

unsigned int ElectionModule::getLeaderID() const {
    std::lock_guard lock(this->leader_id_mutex_);
    return this->leader_id_;
}
