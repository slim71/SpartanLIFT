#include "fixtures.hpp"
#include "types.hpp"

/********************* Simple methods testing **********************/
TEST_F(TacMapTest, TestGetCommanderAck) {
    auto ret = this->core_.getCommanderAck();
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
