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

bool UNSCModule::getRunningStatus() const {
    std::lock_guard<std::mutex> lock(this->running_mutex_);
    return this->running_;
}

void UNSCModule::stopService() {
    std::lock_guard<std::mutex> lock(this->running_mutex_);
    this->running_ = false;
}

Eigen::Vector3f UNSCModule::getInitialOffset() const {
    std::lock_guard<std::mutex> lock(this->offset_mutex_);
    return this->initial_offset_;
}

void UNSCModule::setInitialOffset(float x, float y, float z) {
    std::lock_guard<std::mutex> lock(this->offset_mutex_);
    this->initial_offset_ = {x, y, z};
}
