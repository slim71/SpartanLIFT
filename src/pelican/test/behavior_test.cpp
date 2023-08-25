#include "fixtures.hpp"

TEST_F(PelicanTest, TestRoleCoherence) {
    // Wait just to be sure the node is up and running completely
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // this is a XOR statement: only one must be true
    ASSERT_TRUE((this->node.get()->isLeader() != this->node.get()->isFollower()) != this->node.get()->isCandidate());

}

// TODO: not working for some reason; data not sent if no simulation is run?
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
//         std::this_thread::sleep_for(std::chrono::seconds(1));
//     }
//
// }

TEST_F(PelicanTest, TestHeartbeatPublisher) {
    this->HeartbeatPublisherTester();

    while(!this->node->isLeader()) {

    }

    bool passed = false;
    for(int i=0; i < 10; i++) {
        this->data_ok_mutex.lock();
        bool a = this->data_ok;
        this->data_ok_mutex.unlock();

        if(a) {
            SUCCEED();
            passed = true;
            break;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if(!passed) FAIL();

}

TEST_F(PelicanTest, TestDatapadPublisher) {
    this->DatapadPublisherTester();

    bool passed = false;
    for(int i=0; i < 10; i++) {
        this->data_ok_mutex.lock();
        bool a = this->data_ok;
        this->data_ok_mutex.unlock();

        if(a) {
            SUCCEED();
            passed = true;
            break;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if(!passed) FAIL();

}