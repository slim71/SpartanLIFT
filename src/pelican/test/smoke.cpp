#include "gtest/gtest.h"
#include "pelican.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

TEST(SmokeTests, ParseModelFailing) {
    // Initialize RCLCPP
    rclcpp::init(0, nullptr);

    EXPECT_THROW(
        {
            try {
                PelicanUnit pelican;
            } catch (const std::exception& e) {
                EXPECT_STREQ("Agent model could not be parsed!", e.what());
                throw; // Re-throw the exception for Google Test to catch
            }
        }, 
        std::runtime_error);

    // Shutdown RCLCPP
    rclcpp::shutdown();
}

TEST(SmokeTests, NodeStartingCorrectly) {
    // may throw ament_index_cpp::PackageNotFoundError exception
    std::string pelican_share_directory = ament_index_cpp::get_package_share_directory("pelican");
    std::string config_file_path = pelican_share_directory + "/config/copter_test.yaml";

    char  arg0[] = "--ros-args";
    char  arg1[] = "--params-file";
    char*  arg2 = new char[config_file_path.length()+1];
    strcpy(arg2, config_file_path.c_str());

    char* argv[] = { &arg0[0], &arg1[0], &arg2[0], NULL };
    int   argc   = (int)(sizeof(argv) / sizeof(argv[0])) - 1;

    // Initialization
    rclcpp::init(argc, argv);

    // Instantiation
    rclcpp::Node::SharedPtr node = std::make_shared<PelicanUnit>();
    // Set the instance pointer to the shared pointer of the main node
    PelicanUnit::setInstance(node);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::thread t(
        [&](){
            ASSERT_NO_THROW(executor.spin(););
        }
    );

    std::this_thread::sleep_for(std::chrono::seconds(5));
    executor.cancel();
    t.join(); // Wait for thread completion

    rclcpp::shutdown();
    delete[] arg2;
}