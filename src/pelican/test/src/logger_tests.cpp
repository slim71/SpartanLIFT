#include "fixtures.hpp"

/********************* Simple methods testing **********************/
TEST_F(LoggerTest, TestGetID) {
    int ret;
    ASSERT_NO_THROW(ret = this->core_->getID());
    ASSERT_EQ(ret, 0);
}

TEST_F(LoggerTest, TestSetID) {
    const int desiredID = 2;
    ASSERT_NO_THROW(this->core_->setID(desiredID));
    ASSERT_EQ(this->core_->getID(), desiredID);
}
