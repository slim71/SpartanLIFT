#include "ElectionModule/election.hpp"
#include "HeartbeatModule/heartbeat.hpp"
#include "LoggerModule/logger.hpp"
#include "PelicanModule/pelican.hpp"
#include "gtest/gtest.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

TEST(SmokeTests, ParseModelFailing) {
    // Initialize RCLCPP
    rclcpp::init(0, nullptr);

    EXPECT_THROW(
        {
            try {
                Pelican pelican;
            } catch (const std::exception& e) {
                EXPECT_STREQ("Agent model could not be parsed!", e.what());
                throw; // Re-throw the exception for Google Test to catch
            }
        },
        std::runtime_error
    );

    // Shutdown RCLCPP
    rclcpp::shutdown();
}

TEST(SmokeTests, NodeStartingCorrectly) {
    // May throw ament_index_cpp::PackageNotFoundError exception
    std::string pelican_share_directory = ament_index_cpp::get_package_share_directory("pelican");
    std::string config_file_path = pelican_share_directory + "/config/copter_test.yaml";

    char arg0[] = "--ros-args";
    char arg1[] = "--params-file";
    char* arg2 = new char[config_file_path.length() + 1];
    strcpy(arg2, config_file_path.c_str());

    char* argv[] = {&arg0[0], &arg1[0], &arg2[0], NULL};
    int argc = (int) (sizeof(argv) / sizeof(argv[0])) - 1;

    // Initialization
    rclcpp::init(argc, argv);

    // Instantiation
    rclcpp::Node::SharedPtr node = std::make_shared<Pelican>();
    // Set the instance pointer to the shared pointer of the main node
    Pelican::setInstance(node);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::thread t([&]() {
        ASSERT_NO_THROW(executor.spin(););
    });

    std::this_thread::sleep_for(std::chrono::seconds(1));
    executor.cancel();
    t.join(); // Wait for thread completion

    rclcpp::shutdown();
    delete[] arg2;
}

TEST(SmokeTests, SignalHandling) {
    // may throw ament_index_cpp::PackageNotFoundError exception
    std::string pelican_share_directory = ament_index_cpp::get_package_share_directory("pelican");
    std::string config_file_path = pelican_share_directory + "/config/copter_test.yaml";

    char arg0[] = "--ros-args";
    char arg1[] = "--params-file";
    char* arg2 = new char[config_file_path.length() + 1];
    strcpy(arg2, config_file_path.c_str());

    char* argv[] = {&arg0[0], &arg1[0], &arg2[0], NULL};
    int argc = (int) (sizeof(argv) / sizeof(argv[0])) - 1;

    // Initialization
    rclcpp::init(argc, argv);

    // Instantiation
    rclcpp::Node::SharedPtr node = std::make_shared<Pelican>();
    // Set the instance pointer to the shared pointer of the main node
    Pelican::setInstance(node);
    // Register the signal handler for SIGINT (CTRL+C)
    std::signal(SIGINT, Pelican::signalHandler);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    auto spin_thread = std::thread([&]() {
        ASSERT_NO_THROW(executor.spin(););
    });

    std::this_thread::sleep_for(std::chrono::seconds(1));
    int res = std::raise(SIGINT);
    ASSERT_EQ(res, 0);
    spin_thread.join(); // Wait for thread completion

    rclcpp::shutdown();
    delete[] arg2;
}

TEST(SmokeTests, HeartbeatModuleTest) {
    std::shared_ptr<HeartbeatModule> hb;
    ASSERT_NO_THROW(
        try { hb = std::make_shared<HeartbeatModule>(); } catch (const std::exception& e) {
            EXPECT_STREQ("HeartbeatModule cannot be created!", e.what());
            throw; // Re-throw the exception for Google Test to catch
        }
    );

    // Just to show the successful creation of a working object
    ASSERT_NO_THROW(hb->getNumberOfHbs());
}

TEST(SmokeTests, LoggerModuleTest) {
    std::shared_ptr<LoggerModule> l;
    ASSERT_NO_THROW(
        try {
            l = std::make_shared<LoggerModule>(
                std::make_shared<rclcpp::Logger>(rclcpp::get_logger("TestLogger"))
            );
        } catch (const std::exception& e) {
            EXPECT_STREQ("LoggerModule cannot be created!", e.what());
            throw; // Re-throw the exception for Google Test to catch
        }
    );

    // Just to show the successful creation of a working object
    ASSERT_NO_THROW(l->getID());
}

TEST(SmokeTests, ElectionModuleTest) {
    std::shared_ptr<ElectionModule> e;
    ASSERT_NO_THROW(
        try { e = std::make_shared<ElectionModule>(); } catch (const std::exception& e) {
            EXPECT_STREQ("ElectionModule cannot be created!", e.what());
            throw; // Re-throw the exception for Google Test to catch
        }
    );

    // Just to show the successful creation of a working object
    ASSERT_NO_THROW(e->getLeaderID());
}
