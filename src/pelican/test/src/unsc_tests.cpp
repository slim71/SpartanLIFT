#include "fixtures.hpp"
#include "types.hpp"

/********************* Simple methods testing **********************/
TEST_F(UNSCTest, TestGetRunningStatus) {
    auto ret = this->core_.getRunningStatus();
    ASSERT_EQ(typeid(ret), typeid(bool));
    ASSERT_EQ(ret, true);
}

TEST_F(UNSCTest, TestStopService) {
    ASSERT_NO_THROW(this->core_.stopService());
}
