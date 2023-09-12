#include "fixtures.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>

using rclcpp::get_logger;

/************************** SetUp/TearDown *************************/
void PelicanTest::SetUp() {
    std::string pelican_share_directory = ament_index_cpp::get_package_share_directory("pelican");
    std::string config_file_path = pelican_share_directory + "/config/copter_test.yaml";

    char* argv[] = {
        strdup("--ros-args"), strdup("--params-file"), strdup(config_file_path.c_str()), 
        // strdup("--log-level"), strdup("debug"), 
        NULL
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

    this->spin_thread_ = std::thread([this]() { this->executor_->spin(); });
}

void PelicanTest::TearDown() {
    if(this->executor_)
        this->executor_->cancel();
    this->spin_thread_.join(); // Wait for thread completion
    rclcpp::shutdown();
}

void LoggerTest::SetUp() {
    this->l_ = std::make_shared<rclcpp::Logger>(get_logger("TestLogger"));
    this->core_ = std::make_shared<LoggerModule>(this->l_);
}

/********************** Other member functions *********************/
void PelicanTest::PositionPublisherTester() {
    this->pos_sub_ = this->node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", this->px4_qos_,
        [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(this->data_ok_mutex_);
            this->data_ok_ = true;
        },
        this->node_->getReentrantOptions()
    );
}

void PelicanTest::HeartbeatPublisherTester() {
    this->hb_sub_ = this->node_->create_subscription<comms::msg::Heartbeat>(
        "/fleet/heartbeat", this->standard_qos_,
        [this](const comms::msg::Heartbeat::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(this->data_ok_mutex_);
            this->data_ok_ = true;
        },
        this->node_->getReentrantOptions()
    );
}

void PelicanTest::DatapadPublisherTester() {
    this->data_sub_ = this->node_->create_subscription<comms::msg::Datapad>(
        "/fleet/leader_election", this->standard_qos_,
        [this](const comms::msg::Datapad::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(this->data_ok_mutex_);
            this->data_ok_ = true;
        },
        this->node_->getReentrantOptions()
    );
}
