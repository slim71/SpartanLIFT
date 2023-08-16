#include "fixtures.hpp"

TEST_F(PelicanUnitTest, TestRoleCoherence) {
    // Wait just to be sure the node is up and running completely
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ASSERT_TRUE((this->node.get()->isLeader() != this->node.get()->isFollower()) != this->node.get()->isCandidate());

}