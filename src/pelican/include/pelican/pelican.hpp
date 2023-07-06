#ifndef _PELICAN_HPP_
#define _PELICAN_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <queue>
#include "comms/msg/datapad.hpp"
#include "comms/msg/heartbeat.hpp"

enum possible_roles { tbd, candidate, follower, leader };

struct heartbeat {
    int term;
    int leader;
    rclcpp::Time timestamp;
};

class PelicanUnit : public rclcpp::Node {
    public:
        explicit PelicanUnit();
        ~PelicanUnit();

        int get_ID();
        std::string get_name();
        std::string get_model();
        double get_mass();
        possible_roles get_role();
        int get_current_term();

        bool isLeader();

    // Member functions
    private: 
        void parseModel();
        
        void leaderSelection(const comms::msg::Datapad::SharedPtr msg) const;

        void printData(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) const;

        void becomeLeader();

        void becomeFollower();

        void becomeCandidate();

        void vote(int id_to_vote);

        void randomTimer();

        void sendHeartbeat();

        void requestVote();

        void stopHeartbeat();

        void checkHeartbeat();

        void storeHeartbeat(const comms::msg::Heartbeat msg);

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

        std::string heartbeat_topic_ { "/fleet/heartbeat" };
        rclcpp::Subscription<comms::msg::Heartbeat>::SharedPtr sub_to_heartbeat_topic_;
        rclcpp::Publisher<comms::msg::Heartbeat>::SharedPtr pub_to_heartbeat_topic_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr hb_monitoring_timer_;
        rclcpp::TimerBase::SharedPtr hb_transmission_timer_;
        bool hb_monitoring_running_ {false};
        std::thread heartbeat_thread_ {};

        bool leader_elected_ {false};

        int current_term_ {0};

        std::queue<heartbeat> received_hbs_;

        // rclcpp::Clock system_clock;

        std::mutex hbs_mutex_; // to use with received_hbs

        rclcpp::CallbackGroup::SharedPtr exclusive_group_;
        // Used to allow storeHeartbeat() and checkHeartbeat() to run in parallel
        rclcpp::CallbackGroup::SharedPtr reentrant_group_;
};

#endif