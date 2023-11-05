#include "fixtures.hpp"

using rclcpp::get_logger;

/************************** SetUp/TearDown *************************/
void PelicanTest::SetUp() {
    std::string pelican_share_directory = ament_index_cpp::get_package_share_directory("pelican");
    std::string config_file_path = pelican_share_directory + "/config/copter_test.yaml";

    char* argv[] = {
        strdup("--ros-args"),
        strdup("--params-file"),
        strdup(config_file_path.c_str()),
        // strdup("--log-level"), strdup("debug"),
        NULL,
    };
    int argc = (int) (sizeof(argv) / sizeof(argv[0])) - 1;

    // Initialization
    rclcpp::init(argc, argv);

    // Clean up dynamically allocated memory
    for (int i = 0; i < argc; ++i) {
        free(argv[i]);
    }

    // Instantiation
    this->node_ = std::make_shared<Pelican>();
    // Set the instance pointer to the shared pointer of the main node
    Pelican::setInstance(this->node_);

    this->reentrant_group_ =
        this->node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    this->reentrant_opt_.callback_group = this->reentrant_group_;

    this->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    this->executor_->add_node(this->node_);

    this->spin_thread_ = std::thread([this]() {
        this->executor_->spin();
    });
}

void PelicanTest::TearDown() {
    // Needed to be sure that no operations are pending after cancelling
    // the executor, otherwise the spin() does not return
    if (this->node_) {
        this->node_->commenceStopHeartbeatService();
        this->node_->commenceStopElectionService();
        this->node_->commenceStopTacMapService();
        this->node_->commenceStopUNSCService();
    }

    rclcpp::shutdown();

    this->spin_thread_.join(); // Wait for thread completion
}

void LoggerTest::SetUp() {
    this->l_ = std::make_shared<rclcpp::Logger>(get_logger("TestLogger"));
    this->core_ = std::make_shared<LoggerModule>(this->l_);
}

void TacMapTest::SetUp() {
    // Initialization
    rclcpp::init(0, nullptr);
    this->node_ = std::make_shared<rclcpp::Node>("Tester");
    // Setting up the Reentrant group
    this->reentrant_group_ =
        this->node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    this->reentrant_opt_.callback_group = this->reentrant_group_;
}

void TacMapTest::TearDown() {
    rclcpp::shutdown();
}

/********************** Pelican member functions *********************/
void PelicanTest::HeartbeatPublisherTester() {
    this->hb_sub_ = this->node_->create_subscription<comms::msg::Heartbeat>(
        "/fleet/heartbeat", this->standard_qos_,
        [this](const comms::msg::Heartbeat::SharedPtr) {
            std::lock_guard<std::mutex> lock(this->data_ok_mutex_);
            this->data_ok_ = true;
        },
        this->node_->getReentrantOptions()
    );
}

void PelicanTest::DatapadPublisherTester() {
    this->data_sub_ = this->node_->create_subscription<comms::msg::Datapad>(
        "/fleet/leader_election", this->standard_qos_,
        [this](const comms::msg::Datapad::SharedPtr) {
            std::lock_guard<std::mutex> lock(this->data_ok_mutex_);
            this->data_ok_ = true;
        },
        this->node_->getReentrantOptions()
    );
}

int PelicanTest::RequestNumberOfHbsTester() {
    return this->node_->requestNumberOfHbs();
}

heartbeat PelicanTest::RequestLastHbTester() {
    return this->node_->requestLastHb();
}

std::optional<px4_msgs::msg::VehicleGlobalPosition> PelicanTest::RequestGlobalPositionTester() {
    return this->node_->requestGlobalPosition();
}

std::optional<px4_msgs::msg::VehicleOdometry> PelicanTest::RequestOdometryTester() {
    return this->node_->requestOdometry();
}

std::optional<px4_msgs::msg::VehicleCommandAck> PelicanTest::RequestAckTester() {
    return this->node_->requestAck();
}

std::optional<px4_msgs::msg::VehicleStatus> PelicanTest::RequestStatusTester() {
    return this->node_->requestStatus();
}

void PelicanTest::CommenceFollowerOperationsTester() {
    this->node_->commenceFollowerOperations();
}

void PelicanTest::CommenceCandidateOperationsTester() {
    this->node_->commenceCandidateOperations();
}

void PelicanTest::CommenceLeaderOperationsTester() {
    this->node_->commenceLeaderOperations();
}

void PelicanTest::CommencePublishVehicleCommandTester() {
    this->node_->commencePublishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_RUN_PREARM_CHECKS
    );
}

/********************** TacMap member functions *********************/
void TacMapTest::VehicleCommandPublisherTester() {
    this->command_sub_ = this->node_->create_subscription<px4_msgs::msg::VehicleCommand>(
        "/px4_1/fmu/in/vehicle_command", this->px4_qos_,
        [this](const px4_msgs::msg::VehicleCommand::SharedPtr) {
            std::lock_guard<std::mutex> lock(this->data_ok_mutex_);
            this->data_ok_ = true;
        },
        this->reentrant_opt_
    );

    // Sending the "pre-arm checks" command for the test
    this->core_.publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_RUN_PREARM_CHECKS);
}
