#include "fixtures.hpp"
#include "types.hpp"

/********************* Simple methods testing **********************/

// Can't use this because in ament_cmake_gmock at
// the 'humble' tag ThrowsMessage (as other functions) is not defined
// EXPECT_THAT(
//       [this]() { this->core_.prepareTopics(); },
//       ThrowsMessage<std::runtime_error>(HasSubstr(EXTERNAL_OFF))
// );

TEST_F(ElectionTest, CantPrepareTopics) {
    ASSERT_ANY_THROW(this->core_.prepareTopics());
}

TEST_F(ElectionTest, TestGetLeaderID) {
    ASSERT_EQ(this->core_.getLeaderID(), 0);
}

TEST_F(ElectionTest, TestResetElectiontimer) {
    ASSERT_NO_THROW(this->core_.resetElectionTimer());
}

TEST_F(ElectionTest, TestResetSubscriptions) {
    ASSERT_NO_THROW(this->core_.resetSubscriptions());
}

TEST_F(ElectionTest, TestSetElectionStatus) {
    int desired_leader = 3;
    ASSERT_NO_THROW(this->core_.setElectionStatus(desired_leader));
    ASSERT_EQ(this->core_.getLeaderID(), desired_leader);
}

TEST_F(ElectionTest, TestEmptyStopBallotThread) {
    ASSERT_NO_THROW(this->core_.stopService());
}

TEST_F(ElectionTest, TestFlushVotes) {
    ASSERT_NO_THROW(this->core_.flushVotes());
}
