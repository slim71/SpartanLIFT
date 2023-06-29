#ifndef PELICAN_HPP_
#define PELICAN_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

enum possible_roles { candidate, follower, leader };


class PelicanUnit : public rclcpp::Node {
    public:
        explicit PelicanUnit();
        ~PelicanUnit();

        int get_ID();
        std::string get_name();
        std::string get_model();
        double get_mass();
        possible_roles get_role();

    // Member functions
    private: 
        void parseModel();
        
        void leaderSelection(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) const;

        void printData(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) const;

    // Attributes
    private:
        // TODO: init values
        std::string name_;
        int id_;
        std::string model_;
        double mass_ {0.0};
        
        possible_roles role_ { candidate };
        int leader_id_ {0};
        
        std::string leader_topic_ { "/fleet/leader_selection" };
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_to_leader_topic_;

        std::string local_pos_topic_ { "/fmu/out/vehicle_local_position" };
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_to_local_pos_;
};

#endif