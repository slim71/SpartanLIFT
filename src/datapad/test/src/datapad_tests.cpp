#include "fixtures.hpp"

TEST_F(DatapadTest, NodeReady) {
    ASSERT_TRUE(this->node_->isRunning());
}
