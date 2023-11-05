#include "fixtures.hpp"
#include "types.hpp"

/********************* Simple methods testing **********************/
TEST_F(TacMapTest, TestGetGlobalPosition) {
    auto ret = this->core_.getGlobalPosition();
    ASSERT_EQ(typeid(ret), typeid(std::optional<px4_msgs::msg::VehicleGlobalPosition>));
}

TEST_F(TacMapTest, TestGetOdometry) {
    auto ret = this->core_.getOdometry();
    ASSERT_EQ(typeid(ret), typeid(std::optional<px4_msgs::msg::VehicleOdometry>));
}

TEST_F(TacMapTest, TestGetAck) {
    auto ret = this->core_.getAck();
    ASSERT_EQ(typeid(ret), typeid(std::optional<px4_msgs::msg::VehicleCommandAck>));
}

TEST_F(TacMapTest, TestGetStatus) {
    auto ret = this->core_.getStatus();
    ASSERT_EQ(typeid(ret), typeid(std::optional<px4_msgs::msg::VehicleStatus>));
}

TEST_F(TacMapTest, TestGetRunningStatus) {
    auto ret = this->core_.getRunningStatus();
    ASSERT_EQ(typeid(ret), typeid(bool));
    ASSERT_EQ(ret, true);
}

TEST_F(TacMapTest, TestStopService) {
    ASSERT_NO_THROW(this->core_.stopService());
}
