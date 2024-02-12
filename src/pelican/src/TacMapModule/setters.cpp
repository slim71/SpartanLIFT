#include "TacMapModule/tacmap.hpp"

void TacMapModule::setInitiatedStatus(bool value) {
    std::lock_guard<std::mutex> lock(this->initiated_mutex_);
    this->initiated_ = value;
}
