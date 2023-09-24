#include "fixtures.hpp"
#include <iostream>

TEST_F(PelicanTest, TestRoleCoherence) {
    // Wait just to be sure the node is up and running completely
    while (!this->node_->isReady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // this is a XOR statement: only one must be true
    ASSERT_TRUE(
        (this->node_.get()->isLeader() != this->node_.get()->isFollower()) !=
        this->node_.get()->isCandidate()
    );
}

// FIXME: not working for some reason; data not sent if no simulation is running?
// TEST_F(PelicanTest, TestPositionPublisher) {
//     this->PositionPublisherTester();
//
//     for(int i=0; i < 10; i++) {
//         this->data_ok_mutex.lock();
//         bool a = this->data_ok;
//         this->data_ok_mutex.unlock();
//
//         EXPECT_TRUE(a);
//         if(a) break;
//
//         std::this_thread::sleep_for(std::chrono::milliseconds(10));
//     }
//
// }

TEST_F(PelicanTest, TestHeartbeatPublisher) {
    this->HeartbeatPublisherTester();

    while (!this->node_->isLeader()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    bool passed = false;
    for (int i = 0; i < 10; i++) {
        this->data_ok_mutex_.lock();
        bool a = this->data_ok_;
        this->data_ok_mutex_.unlock();

        if (a) {
            SUCCEED();
            passed = true;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!passed)
        FAIL();
}

TEST_F(PelicanTest, TestDatapadPublisher) {
    this->DatapadPublisherTester();

    bool passed = false;
    for (int i = 0; i < 10; i++) {
        this->data_ok_mutex_.lock();
        bool a = this->data_ok_;
        this->data_ok_mutex_.unlock();

        if (a) {
            SUCCEED();
            passed = true;
            break;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if (!passed)
        FAIL();
}

TEST_F(PelicanTest, TestRequestNumberOfHbs) {
    // Wait just to be sure the node is up and running completely
    while (!this->node_->isReady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    int ret;
    ASSERT_NO_THROW(ret = this->RequestNumberOfHbsTester());
    ASSERT_EQ(typeid(ret), typeid(int));
    ASSERT_GE(ret, 0);
}

TEST_F(PelicanTest, TestRequestLastHb) {
    // Wait just to be sure the node is up and running completely
    while (!this->node_->isReady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    heartbeat ret;
    ASSERT_NO_THROW(ret = this->RequestLastHbTester());
    ASSERT_EQ(typeid(ret), typeid(heartbeat));
    ASSERT_GE(ret.term, -1);
    ASSERT_GE(ret.leader, -1);
    ASSERT_GE(ret.timestamp, rclcpp::Time(0, 0));
}

TEST_F(PelicanTest, TestCommenceFollowerOperations) {
    // Wait just to be sure the node is up and running completely
    while (!this->node_->isReady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_NO_THROW(this->CommenceFollowerOperationsTester());
    bool ret = this->node_->isFollower();
    ASSERT_TRUE(ret);
}

TEST_F(PelicanTest, DISABLED_TestCommenceCandidateOperations) {
    // Wait just to be sure the node is up and running completely
    while (!this->node_->isReady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_NO_THROW(this->CommenceCandidateOperationsTester());
    bool ret = this->node_->isCandidate();
    ASSERT_TRUE(ret);
}

TEST_F(PelicanTest, TestCommenceLeaderOperations) {
    // Wait just to be sure the node is up and running completely
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ASSERT_NO_THROW(this->CommenceLeaderOperationsTester());
    bool ret = this->node_->isLeader();
    ASSERT_TRUE(ret);
}
