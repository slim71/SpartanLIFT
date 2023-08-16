#include "gtest/gtest.h"
#include <rclcpp/rclcpp.hpp>
#include "pelican.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// int main(int argc, char** argv) {
//     // may throw ament_index_cpp::PackageNotFoundError exception
//     std::string pelican_share_directory = ament_index_cpp::get_package_share_directory("pelican");
//     std::string config_file_path = pelican_share_directory + "/config/copter_test.yaml";

//     char  myarg0[] = "--ros-args";
//     char  myarg1[] = "--params-file";
//     char*  myarg2 = new char[config_file_path.length()+1];
//     strcpy(myarg2, config_file_path.c_str());

//     char* myargv[] = { &myarg0[0], &myarg1[0], &myarg2[0], NULL };
//     int   myargc   = (int)(sizeof(myargv) / sizeof(myargv[0])) - 1;

//     // Initialization
//     rclcpp::init(myargc, myargv);
//     // Initialize GTest
//     ::testing::InitGoogleTest(&argc, argv);
//     // Run tests
//     int result = RUN_ALL_TESTS();

//     // Instantiation
//     rclcpp::Node::SharedPtr node = std::make_shared<PelicanUnit>();
//     // Set the instance pointer to the shared pointer of the main node
//     PelicanUnit::setInstance(node);

//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(node);
//     executor.spin();

//     // std::this_thread::sleep_for(std::chrono::seconds(5));
//     // executor.cancel();
//     // t.join(); // Wait for thread completion


//     // Shutdown RCLCPP
//     rclcpp::shutdown();
//     delete[] myarg2;
    
//     return result;
// }