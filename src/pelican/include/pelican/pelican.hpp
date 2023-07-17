#ifndef _PELICAN_HPP_
#define _PELICAN_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <queue>
#include <iostream>
#include <chrono>
#include <random>
#include "comms/msg/datapad.hpp"
#include "comms/msg/heartbeat.hpp"
#include "comms/msg/request_vote_rpc.hpp"
#include <fmt/core.h>

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

        int getID() const;
        std::string getName() const;
        std::string getModel() const;
        double getMass() const;
        possible_roles getRole() const;
        int getCurrentTerm() const;

        bool isLeader();

    // Member functions
    private:
        template<typename... Args> void logInfo(std::string s, Args... args);
        template<typename... Args> void logError(std::string s, Args... args);
        template<typename... Args> void logWarning(std::string s, Args... args);
        template<typename... Args> void logDebug(std::string s, Args... args);

        // TODO: If a member doesn't modify any member data, mark it const by default.

        void parseModel();
        
        void storeCandidacy(const comms::msg::Datapad::SharedPtr msg);

        void flushVotes();

        void printData(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) const;

        void becomeLeader();

        void becomeFollower();

        void becomeCandidate();

        void vote(int id_to_vote);

        void sendHeartbeat();

        void requestVote();

        void serveVoteRequest(const comms::msg::RequestVoteRPC msg);

        bool checkElectionTimedOut();

        void setElectionTimedOut();

        void unsetElectionTimedOut();

        void resetVotingWindow();

        void leaderElection();

        void setElectionCompleted();

        void unsetElectionCompleted();

        bool checkElectionCompleted();

        void setVotingCompleted();

        void unsetVotingCompleted();

        bool checkVotingCompleted();

        void stopHeartbeat();

        // void checkHeartbeat();

        void storeHeartbeat(const comms::msg::Heartbeat msg);

        void ballotCheckingThread();

        // void externalCandidateWon(); // TODO: not needed?

        bool checkForExternalLeader();

        bool checkExternalLeaderElected();

        void setExternalLeaderElected();

        void unsetExternalLeaderElected();

        void setRandomElectionTimeout();

        void setRandomBallotWaittime();

        bool checkLeaderElected();

        void setLeaderElected();

        void unsetLeaderElected();

        void setLeader(int id);

        void setRole(possible_roles r);

        void increaseCurrentTerm();

        void setMass(double m);

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

        // The subscription sets a QoS profile based on rmw_qos_profile_sensor_data. 
        // This is needed because the default ROS 2 QoS profile for subscribers is 
        // incompatible with the PX4 profile for publishers.
        rclcpp::QoS px4_qos_ {rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, 5), 
                                           rmw_qos_profile_sensor_data
                                          )
                             };
        rmw_qos_profile_t qos_profile_ {rmw_qos_profile_default};
        rclcpp::QoS standard_qos_ {rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile_), qos_profile_)};

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr hb_monitoring_timer_;
        rclcpp::TimerBase::SharedPtr hb_transmission_timer_;
        rclcpp::TimerBase::SharedPtr election_timer_;
        rclcpp::TimerBase::SharedPtr voting_timer;

        bool leader_elected_ {false};
        bool external_leader_elected_ {false}; // TODO: needed different from previous line?
        bool election_completed_ {false}; // candidate (current node) has won an election
        bool election_timed_out_ {false}; // the max time in which a follower must receive a heartbeat has passed
        bool voting_completed_ {false}; // the appropriate amount of time after a vote to judge if all agents voted has passed

        // Wondered if using a std::deque would be better, but in the end I don't expect many items here
        std::vector<heartbeat> received_hbs_;
        
        // TODO: check if SharedPtr works
        std::vector<comms::msg::Datapad::SharedPtr> received_votes_;

        // TODO: macro for single lock/unlock?
        std::mutex hbs_mutex_; // to use with received_hbs
        std::mutex votes_mutex_; // to use with received_votes_
        std::mutex election_timedout_mutex_; // to use with election_timed_out_
        std::mutex election_completed_mutex_; // to use with election_completed_
        std::mutex voting_completed_mutex_; // to use with voting_completed_
        std::mutex candidate_mutex_; // to use when the node is a candidate and tries to win an election
        std::mutex external_leader_mutex_;
        std::mutex leader_mutex_;

        rclcpp::SubscriptionOptions reentrant_opt_ {rclcpp::SubscriptionOptions()};
        // rclcpp::SubscriptionOptions exclusive_opt_;
        // rclcpp::SubscriptionOptions vote_exclusive_opt_;
        // rclcpp::CallbackGroup::SharedPtr exclusive_group_;
        // rclcpp::CallbackGroup::SharedPtr vote_exclusive_group_;
        rclcpp::CallbackGroup::SharedPtr reentrant_group_;

        // Time frame after which a candidacy will begin if no heartbeat from a leader has been received.
        // Raft uses randomized election timeouts to ensure that split votes are rare and that they are resolved quickly
        std::chrono::milliseconds election_timeout_;
        
        // "election_timeout_" is thes same time window used by a follower to decide whether or not to candidate as leader.
        // using another variable only for clarity purposes
        std::chrono::milliseconds new_ballot_waittime_;

        std::chrono::milliseconds heartbeat_period_ {10}; // TODO: value?

        // std::chrono::seconds hb_monitoring_period_ {10}; // TODO: not needed?

        std::chrono::seconds voting_max_time_ {10};

        std::condition_variable cv;

        std::mt19937 random_engine_ { std::random_device{}() }; // mersenne_twister_engine seeded with random_device()
        // TODO: range? (e.g., 150–300ms) is uniform distribution ok?
        std::uniform_int_distribution<> random_distribution_ {150, 300}; // inclusive; intended as milliseconds 

};

// Including templates definitions
#include "pelican_logger.tpp"

#endif