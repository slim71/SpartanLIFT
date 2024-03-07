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

void UNSCModule::stopService() {
    std::lock_guard<std::mutex> lock(this->running_mutex_);
    this->running_ = false;
}

/***************************** Getters *****************************/

bool UNSCModule::getRunningStatus() const {
    std::lock_guard<std::mutex> lock(this->running_mutex_);
    return this->running_;
}

Eigen::Vector3f UNSCModule::getOffset() const {
    std::lock_guard<std::mutex> lock(this->offset_mutex_);
    return this->offset_;
}

/**************************** Utilities ****************************/

Eigen::Vector3f UNSCModule::convertLocalToBody(const Eigen::Vector3f& enu_pos) const {
    // Current-axis composition: 1st_rotation * ... * nth_rotation
    Eigen::Matrix3f rot_ENU2NED = rotZ(-90, true) * rotY(180, true);

    Eigen::Vector3f pos_offset_ENU(this->getOffset().data());

    // p_NED = - R_ENU2NED * p_ENU
    Eigen::Vector3f pos_offset_NED = -rot_ENU2NED.transpose() * pos_offset_ENU;
    Eigen::Vector4f hom_offset_NED = pos_offset_NED.homogeneous();

    Eigen::Vector4f hom_enu_pos = enu_pos.homogeneous();

    Eigen::Matrix4f hom_ENU2NED;
    hom_ENU2NED.block(0, 0, 3, 3) = rot_ENU2NED;
    hom_ENU2NED.block(3, 0, 1, 3) << 0, 0, 0;
    hom_ENU2NED.col(3) << hom_offset_NED;

    // The yaw rotation is thought to be along the Z_ENU axis
    // Fixed-axis composition: nth_rotation * ... * 1st_rotation
    // Here the first rotation is the one re-aligning the rotated vehicle to the ENU frame
    Eigen::Vector4f body_pos = hom_ENU2NED * hom_enu_pos;

    return body_pos.head(3);
}
