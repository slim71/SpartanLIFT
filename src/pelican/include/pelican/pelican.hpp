#ifndef _PELICAN_HPP_
#define _PELICAN_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <queue>
#include <iostream>
#include "comms/msg/datapad.hpp"
#include "comms/msg/heartbeat.hpp"
#include "comms/msg/request_vote_rpc.hpp"

enum possible_roles { tbd, candidate, follower, leader };

struct vote_count {
    int candidate_id;
    int total;
};

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
        // TODO: If a member doesn't modify any member data, mark it const by default.

        void parseModel();
        
        void storeCandidacy(const comms::msg::Datapad::SharedPtr msg);

        void flushVotes();

        void printData(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) const;

        void becomeLeader();

        void becomeFollower();

        void becomeCandidate();

        void vote(int id_to_vote);

        void randomTimer();

        void sendHeartbeat();

        void requestVote();

        void expressPreference(const comms::msg::RequestVoteRPC msg);

        bool checkVotingTimedOut();

        void setVotingTimedOut();

        void resetVotingWindow();

        void leaderElection();

        void setElectionCompleted();

        bool checkElectionCompleted();

        void setVotingCompleted();

        bool checkVotingCompleted();

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

        int current_term_ {0};
        
        std::string leader_selection_topic_ { "/fleet/leader_selection" };
        rclcpp::Subscription<comms::msg::Datapad>::SharedPtr sub_to_leader_selection_topic_;
        rclcpp::Publisher<comms::msg::Datapad>::SharedPtr pub_to_leader_selection_topic_;

        std::string local_pos_topic_ { "/fmu/out/vehicle_local_position" };
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_to_local_pos_topic_;

        std::string heartbeat_topic_ { "/fleet/heartbeat" };
        rclcpp::Subscription<comms::msg::Heartbeat>::SharedPtr sub_to_heartbeat_topic_;
        rclcpp::Publisher<comms::msg::Heartbeat>::SharedPtr pub_to_heartbeat_topic_;

        std::string request_vote_rpc_topic_ { "/fleet/request_vote_rpc" };
        rclcpp::Publisher<comms::msg::RequestVoteRPC>::SharedPtr pub_to_request_vote_rpc_topic_;
        rclcpp::Subscription<comms::msg::RequestVoteRPC>::SharedPtr sub_to_request_vote_rpc_topic_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr hb_monitoring_timer_;
        rclcpp::TimerBase::SharedPtr hb_transmission_timer_;
        rclcpp::TimerBase::SharedPtr election_timer_;
        rclcpp::TimerBase::SharedPtr voting_timer;

        bool leader_elected_ {false};
        bool election_completed_ {false};
        bool election_timed_out {false};
        bool voting_completed_ {false};

        std::queue<heartbeat> received_hbs_;
        // Wondered if using a std::deque would be better, but in the end I don't expect many items here
        // TODO: check if SharedPtr works
        std::vector<comms::msg::Datapad::SharedPtr> received_votes;

        // rclcpp::Clock system_clock;

        std::mutex hbs_mutex_; // to use with received_hbs
        std::mutex votes_mutex_; // to use with received_votes
        std::mutex election_timedout_mutex_; // to use with election_timed_out
        std::mutex election_completed_mutex_; // to use with election_completed_
        std::mutex voting_completed_mutex_; // to use with voting_completed_

        rclcpp::CallbackGroup::SharedPtr exclusive_group_;
        // Used to allow storeHeartbeat() and checkHeartbeat() to run in parallel
        rclcpp::CallbackGroup::SharedPtr reentrant_group_;

        std::chrono::seconds election_max_time_ {10};
        std::chrono::seconds voting_max_time_ {10};

        bool leader_selected {false};
};

#endif