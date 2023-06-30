#ifndef _PELICAN_HPP_
#define _PELICAN_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include "comms/msg/datapad.hpp"

enum possible_roles { tbd, candidate, follower, leader };


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
        
        void leaderSelection(const comms::msg::Datapad::SharedPtr msg) const;

        void printData(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) const;

        void becomeLeader();

        void becomeFollower();

        void becomeCandidate();

        void vote(int id_to_vote);

    // Attributes
    private:
        // TODO: init values
        std::string name_;
        int id_;
        std::string model_;
        double mass_ {0.0};
        
        possible_roles role_ { tbd };
        int leader_id_ {0};
        
        std::string leader_selection_topic_ { "/fleet/leader_selection" };
        rclcpp::Subscription<comms::msg::Datapad>::SharedPtr sub_to_leader_selection_topic_;
        rclcpp::Publisher<comms::msg::Datapad>::SharedPtr pub_to_leader_selection_topic_;

        std::string local_pos_topic_ { "/fmu/out/vehicle_local_position" };
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_to_local_pos_topic_;

        bool leader_elected_ {false};

        int current_term_ {0};
};

#endif