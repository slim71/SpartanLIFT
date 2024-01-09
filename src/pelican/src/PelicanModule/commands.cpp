#include "PelicanModule/pelican.hpp"

void Pelican::rogerWillCo(
    const std::shared_ptr<comms::srv::FleetInfoExchange::Request> request,
    const std::shared_ptr<comms::srv::FleetInfoExchange::Response> response
) {
    response->leader_id = this->id_;
    response->ack = true;

    if (request->presence) {
        this->sendLogDebug("Notifying I'm the leader to the user!");
        response->present = true;
        response->taking_off = false;
        response->landing = false;
    }

    if (request->takeoff) {
        // TODO: send command to other agents; TODO: check if it can be done
        this->sendLogDebug("Notifying start of takeoff!");
        response->present = false;
        response->taking_off = true;
        response->landing = false;
    }

    if (request->landing) {
        // TODO: send command to other agents; TODO: check if it can be done
        this->sendLogDebug("Notifying start of landing operations!");
        response->present = false;
        response->taking_off = false;
        response->landing = true;
    }
}
