#include "fixtures.hpp"
#include "types.hpp"
#include <gmock/gmock-matchers.h>
#include <gmock/gmock.h>

/********************* Simple methods testing **********************/

// Can't use this because in ament_cmake_gmock at 
// the 'humble' tag ThrowsMessage (as other functions) is not defined
// EXPECT_THAT(
//       [this]() { this->core_.setupPublisher(); },
//       ThrowsMessage<std::runtime_error>(HasSubstr("EXTERNAL_OFF"))
// );

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

/************************ Complex behaviors ************************/

