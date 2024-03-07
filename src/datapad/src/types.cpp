#include "types.hpp"

/**************************** Aggregates ***************************/
Flags::Flags() {};

Flags& Flags::SetPresence() {
    this->presence = true;
    return *this;
}

Flags& Flags::SetTakeoff() {
    this->takeoff = true;
    return *this;
}

Flags& Flags::SetLanding() {
    this->landing = true;
    return *this;
}

Flags& Flags::SetRetrieval() {
    this->retrieval = true;
    return *this;
}

Flags& Flags::SetDropoff() {
    this->dropoff = true;
    return *this;
}

bool Flags::GetPresence() const {
    return this->presence;
}

bool Flags::GetTakeoff() const {
    return this->takeoff;
}

bool Flags::GetLanding() const {
    return this->landing;
}

bool Flags::GetRetrieval() const {
    return this->retrieval;
}

bool Flags::GetDropoff() const {
    return this->dropoff;
}
