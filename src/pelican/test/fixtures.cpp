#include "fixtures.hpp"

void PelicanTest::SetUp() {
    std::string pelican_share_directory = ament_index_cpp::get_package_share_directory("pelican");
    std::string config_file_path = pelican_share_directory + "/config/copter_test.yaml";

    char* argv[] = {
        strdup("--ros-args"),
        strdup("--params-file"),
        strdup(config_file_path.c_str()),
        NULL
    };
    int   argc   = (int)(sizeof(argv) / sizeof(argv[0])) - 1;

    // Initialization
    rclcpp::init(argc, argv);

    // Clean up dynamically allocated memory
    for (int i = 0; i < argc; ++i) {
        free(argv[i]);
    }

    // Instantiation
    this->node = std::make_shared<Pelican>();
    // Set the instance pointer to the shared pointer of the main node
    Pelican::setInstance(this->node);

    this->reentrant_group_ = this->node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    this->reentrant_opt_.callback_group = this->reentrant_group_;

    this->executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    this->executor->add_node(this->node);

    this->spin_thread = std::thread([this](){this->executor->spin();});
}

void PelicanTest::TearDown() {
    this->executor->cancel();
    this->spin_thread.join(); // Wait for thread completion
    rclcpp::shutdown();
}

void PelicanTest::PositionPublisherTester() {

    this->pos_sub = this->node->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", this->px4_qos_, 
        [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) { 
            std::lock_guard<std::mutex> lock(this->data_ok_mutex);
            this->data_ok = true; 
        },
        this->reentrant_opt_);
}

void PelicanTest::HeartbeatPublisherTester() {

    this->hb_sub = this->node->create_subscription<comms::msg::Heartbeat>(
        "/fleet/heartbeat", this->standard_qos_, 
        [this](const comms::msg::Heartbeat::SharedPtr msg) { 
            std::lock_guard<std::mutex> lock(this->data_ok_mutex);
            this->data_ok = true; 
        },
        this->reentrant_opt_);
}

void PelicanTest::DatapadPublisherTester() {

    this->data_sub = this->node->create_subscription<comms::msg::Datapad>(
        "/fleet/leader_election", this->standard_qos_, 
        [this](const comms::msg::Datapad::SharedPtr msg) { 
            std::lock_guard<std::mutex> lock(this->data_ok_mutex);
            this->data_ok = true; 
        },
        this->reentrant_opt_);
}