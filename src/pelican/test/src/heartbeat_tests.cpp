#include "fixtures.hpp"
#include "types.hpp"

/********************* Simple methods testing **********************/
TEST_F(HeartbeatTest, CannotSetupPublisher) {
    ASSERT_ANY_THROW(this->core_.setupPublisher());
}

TEST_F(HeartbeatTest, CannotSetupSubscription) {
    ASSERT_ANY_THROW(this->core_.setupSubscription());
}

TEST_F(HeartbeatTest, CannotSetupTransmissionTimer) {
    ASSERT_ANY_THROW(this->core_.setupTransmissionTimer());
}

TEST_F(HeartbeatTest, TestResetSubscription) {
    ASSERT_NO_THROW(this->core_.resetSubscription());
}

TEST_F(HeartbeatTest, TestResetPublisher) {
    ASSERT_NO_THROW(this->core_.resetPublisher());
}

TEST_F(HeartbeatTest, TestFlushHeartbeats) {
    ASSERT_NO_THROW(this->core_.flushHeartbeats());
    ASSERT_EQ(this->core_.getNumberOfHbs(), 0);
}

TEST_F(HeartbeatTest, TestGetMaxHbs) {
    auto ret = this->core_.getMaxHbs();
    std::cout << "MaxHbs: " << ret << std::endl;
    ASSERT_EQ(typeid(ret), typeid(int));
    ASSERT_GE(ret, 0);
}

TEST_F(HeartbeatTest, TestGetNumberOfHbs) {
    auto ret = this->core_.getNumberOfHbs();
    std::cout << "Hbs number: " << ret << std::endl;
    ASSERT_EQ(typeid(ret), typeid(int));
    ASSERT_GE(ret, 0);
}

TEST_F(HeartbeatTest, TestGetLastHb) {
    auto ret = this->core_.getLastHb();
    std::cout << "Last Hbs --> " << ret << std::endl;
    ASSERT_EQ(typeid(ret), typeid(heartbeat));
}

TEST_F(HeartbeatTest, CannotSendHeartbeat) {
    ASSERT_ANY_THROW(this->core_.sendNow());
}

TEST_F(HeartbeatTest, TestStopService) {
    ASSERT_NO_THROW(this->core_.stopService());
}
