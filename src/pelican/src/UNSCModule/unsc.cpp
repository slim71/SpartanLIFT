#include "UNSCModule/unsc.hpp"
#include "PelicanModule/pelican.hpp"

/************************** Ctors/Dctors ***************************/
UNSCModule::UNSCModule() {
    this->node_ = nullptr;
    this->logger_ = nullptr;
}

UNSCModule::UNSCModule(Pelican* node) : node_(node), logger_ {nullptr} {}

UNSCModule::~UNSCModule() {
    this->node_ = nullptr;
    this->logger_ = nullptr;
}

/************************** Setup methods **************************/

void UNSCModule::initSetup(LoggerModule* logger) {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->logger_ = logger;

    // Allow for some loading time before starting all operations...
    this->starting_timer_ = this->node_->create_wall_timer(
        this->briefing_time_, std::bind(&UNSCModule::runPreChecks, this),
        this->gatherReentrantGroup()
    );
}

bool UNSCModule::getRunningStatus() {
    std::lock_guard<std::mutex> lock(this->running_mutex_);
    return this->running_;
}

void UNSCModule::stopService() {
    std::lock_guard<std::mutex> lock(this->running_mutex_);
    this->running_ = false;
}
