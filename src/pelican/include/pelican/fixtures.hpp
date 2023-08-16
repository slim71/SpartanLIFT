#include "gtest/gtest.h"
#include "pelican.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

class PelicanUnitTest : public ::testing::Test {
    protected:
        std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
        std::shared_ptr<PelicanUnit> node;
        std::thread spin_thread;

        void SetUp() override;
        void TearDown() override;
};