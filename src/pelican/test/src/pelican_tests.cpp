#include "fixtures.hpp"

/********************* Simple methods testing **********************/
TEST_F(PelicanTest, TestGetID) {
    // Wait just to be sure the node is up and running completely
    std::this_thread::sleep_for(std::chrono::seconds(1));
    auto ret = this->node_.get()->getID();
    std::cout << "ID: " << ret << std::endl;
    ASSERT_EQ(typeid(this->node_.get()->getID()), typeid(int));
    ASSERT_GT(ret, 0);
    ASSERT_EQ(ret, 1);
}

TEST_F(PelicanTest, TestGetModel) {
    // Wait just to be sure the node is up and running completely
    std::this_thread::sleep_for(std::chrono::seconds(1));
    auto ret = this->node_.get()->getModel();
    std::cout << "Model: " << ret << std::endl;
    ASSERT_EQ(typeid(ret), typeid(std::string));
    ASSERT_EQ(ret, "/home/slim71/Documents/git/SpartanLIFT/src/pelican/models/X3/model.sdf");
}

TEST_F(PelicanTest, TestGetMass) {
    // Wait just to be sure the node is up and running completely
    std::this_thread::sleep_for(std::chrono::seconds(1));
    auto ret = this->node_.get()->getMass();
    std::cout << "Mass: " << ret << std::endl;
    ASSERT_EQ(typeid(ret), typeid(double));
    ASSERT_GT(ret, 0);
}

TEST_F(PelicanTest, TestGetRole) {
    // Wait just to be sure the node is up and running completely
    std::this_thread::sleep_for(std::chrono::seconds(1));
    auto ret = this->node_.get()->getRole();
    std::cout << "Role: " << roles_to_string(ret) << " (" << ret << ")" << std::endl;
    ASSERT_EQ(typeid(ret), typeid(possible_roles::tbd));
}

TEST_F(PelicanTest, TestGetCurrentTerm) {
    // Wait just to be sure the node is up and running completely
    std::this_thread::sleep_for(std::chrono::seconds(1));
    auto ret = this->node_.get()->getCurrentTerm();
    std::cout << "Term: " << ret << std::endl;
    ASSERT_EQ(typeid(ret), typeid(int));
    ASSERT_GE(ret, (unsigned int) 0);
}

TEST_F(PelicanTest, TestGetReentrantOptions) {
    // Wait just to be sure the node is up and running completely
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ASSERT_EQ(
        typeid(this->node_.get()->getReentrantOptions()), typeid(rclcpp::SubscriptionOptions)
    );
}

TEST_F(PelicanTest, TestGetReentrantGroup) {
    // Wait just to be sure the node is up and running completely
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ASSERT_EQ(
        typeid(this->node_.get()->getReentrantGroup()), typeid(rclcpp::CallbackGroup::SharedPtr)
    );
}

TEST_F(PelicanTest, TestGetInstance) {
    // Wait just to be sure the node is up and running completely
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ASSERT_EQ(typeid(this->node_.get()->getInstance()), typeid(std::shared_ptr<Pelican>));
}

TEST_F(PelicanTest, TestGetTime) {
    // Wait just to be sure the node is up and running completely
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ASSERT_EQ(typeid(this->node_.get()->getTime()), typeid(rclcpp::Time));
}
