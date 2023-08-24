#include "fixtures.hpp"

TEST_F(PelicanTest, TestGetID) {
    // Wait just to be sure the node is up and running completely
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "ID: " << this->node.get()->getID() << std::endl;
    ASSERT_EQ(typeid(this->node.get()->getID()), typeid(int));
    ASSERT_GT(this->node.get()->getID(), 0);
    ASSERT_EQ(this->node.get()->getID(), 1);
}

TEST_F(PelicanTest, TestGetModel) {
    // Wait just to be sure the node is up and running completely
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Model: " << this->node.get()->getModel() << std::endl;
    ASSERT_EQ(typeid(this->node.get()->getModel()), typeid(std::string));
    ASSERT_EQ(this->node.get()->getModel(), "/home/slim71/Documents/git/SpartanLIFT/src/pelican/models/X3/model.sdf");
}

TEST_F(PelicanTest, TestGetMass) {
    // Wait just to be sure the node is up and running completely
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Mass: " << this->node.get()->getMass() << std::endl;
    ASSERT_EQ(typeid(this->node.get()->getMass()), typeid(double));
    ASSERT_GT(this->node.get()->getMass(), 0);
}

TEST_F(PelicanTest, TestGetCurrentTerm) {
    // Wait just to be sure the node is up and running completely
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Term: " << this->node.get()->getCurrentTerm() << std::endl;
    ASSERT_EQ(typeid(this->node.get()->getCurrentTerm()), typeid(int));
    ASSERT_GE(this->node.get()->getCurrentTerm(), 0);
}

TEST_F(PelicanTest, TestGetInstance) {
    // Wait just to be sure the node is up and running completely
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ASSERT_EQ(typeid(this->node.get()->getInstance()), typeid(std::shared_ptr<Pelican>));
}

TEST_F(PelicanTest, TestGetMaxHbs) {
    // Wait just to be sure the node is up and running completely
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "MaxHbs: " << this->node.get()->getMaxHbs() << std::endl;
    ASSERT_EQ(typeid(this->node.get()->getMaxHbs()), typeid(int));
    ASSERT_GE(this->node.get()->getMaxHbs(), 0);
}

TEST_F(PelicanTest, TestGetRole) {
    // Wait just to be sure the node is up and running completely
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Role: " << this->node.get()->getRole() << std::endl;
    ASSERT_EQ(typeid(this->node.get()->getRole()), typeid(possible_roles::tbd));
}
