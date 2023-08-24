#include "fixtures.hpp"

void PelicanTest::SetUp() {
    std::string pelican_share_directory = ament_index_cpp::get_package_share_directory("pelican");
    std::string config_file_path = pelican_share_directory + "/config/copter_test.yaml";
    std::cout << "config_file_path=" << config_file_path << std::endl;

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

    this->executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    this->executor->add_node(this->node);

    this->spin_thread = std::thread([this](){this->executor->spin();});
};

void PelicanTest::TearDown() {
    this->executor->cancel();
    this->spin_thread.join(); // Wait for thread completion
    rclcpp::shutdown();
};
