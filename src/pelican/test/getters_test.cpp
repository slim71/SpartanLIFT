#include "fixtures.hpp"

TEST_F(PelicanTest, TestGetID) {
    // Wait just to be sure the node is up and running completely
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "ID: " << this->node_.get()->getID() << std::endl;
    ASSERT_EQ(typeid(this->node_.get()->getID()), typeid(int));
    ASSERT_GT(this->node_.get()->getID(), 0);
    ASSERT_EQ(this->node_.get()->getID(), 1);
}

TEST_F(PelicanTest, TestGetModel) {
    // Wait just to be sure the node is up and running completely
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Model: " << this->node_.get()->getModel() << std::endl;
    ASSERT_EQ(typeid(this->node_.get()->getModel()), typeid(std::string));
    ASSERT_EQ(this->node_.get()->getModel(), "/home/slim71/Documents/git/SpartanLIFT/src/pelican/models/X3/model.sdf");
}

TEST_F(PelicanTest, TestGetMass) {
    // Wait just to be sure the node is up and running completely
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Mass: " << this->node_.get()->getMass() << std::endl;
    ASSERT_EQ(typeid(this->node_.get()->getMass()), typeid(double));
    ASSERT_GT(this->node_.get()->getMass(), 0);
}

TEST_F(PelicanTest, TestGetCurrentTerm) {
    // Wait just to be sure the node is up and running completely
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Term: " << this->node_.get()->getCurrentTerm() << std::endl;
    ASSERT_EQ(typeid(this->node_.get()->getCurrentTerm()), typeid(int));
    ASSERT_GE(this->node_.get()->getCurrentTerm(), 0);
}

TEST_F(PelicanTest, TestGetInstance) {
    // Wait just to be sure the node is up and running completely
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ASSERT_EQ(typeid(this->node_.get()->getInstance()), typeid(std::shared_ptr<Pelican>));
}

// TODO: move to HeartbeatModule tests
// TEST_F(PelicanTest, TestGetMaxHbs) {
//     // Wait just to be sure the node is up and running completely
//     std::this_thread::sleep_for(std::chrono::seconds(1));
//     std::cout << "MaxHbs: " << this->node_.get()->getMaxHbs() << std::endl;
//     ASSERT_EQ(typeid(this->node_.get()->getMaxHbs()), typeid(int));
//     ASSERT_GE(this->node_.get()->getMaxHbs(), 0);
// }

TEST_F(PelicanTest, TestGetRole) {
    // Wait just to be sure the node is up and running completely
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Role: " << roles_to_string(this->node_.get()->getRole()) << " (" << this->node_.get()->getRole() << ")" << std::endl;
    ASSERT_EQ(typeid(this->node_.get()->getRole()), typeid(possible_roles::tbd));
}
