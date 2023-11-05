#include "fixtures.hpp"
#include "types.hpp"

/********************* Simple methods testing **********************/
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
    // mersenne_twister_engine seeded with random_device()
    std::mt19937 random_engine_ {std::random_device {}()};
    std::uniform_int_distribution<> random_distribution_ {1, 10};
    int desired_leader = random_distribution_(random_engine_);
    ASSERT_NO_THROW(this->core_.setElectionStatus(desired_leader));
    ASSERT_EQ(this->core_.getLeaderID(), desired_leader);
}

TEST_F(ElectionTest, TestStopService) {
    ASSERT_NO_THROW(this->core_.stopService());
}

TEST_F(ElectionTest, TestFlushVotes) {
    ASSERT_NO_THROW(this->core_.flushVotes());
}
