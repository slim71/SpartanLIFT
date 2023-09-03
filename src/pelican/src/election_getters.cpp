#include "election.hpp"

std::chrono::milliseconds ElectionModule::getBallotWaitTime() const {
    return this->new_ballot_waittime_;
}
