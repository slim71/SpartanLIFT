#include "fixtures.hpp"

TEST_F(PelicanTest, TestRoleCoherence) {
    // Wait just to be sure the node is up and running completely
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // this is a XOR statement: only one must be true
    ASSERT_TRUE((this->node.get()->isLeader() != this->node.get()->isFollower()) != this->node.get()->isCandidate());

}