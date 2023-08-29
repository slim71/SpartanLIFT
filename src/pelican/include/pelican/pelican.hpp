#ifndef _PELICAN_HPP_
#define _PELICAN_HPP_

#include "comms/msg/datapad.hpp"
#include "comms/msg/request_vote_rpc.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include <chrono>
#include <iostream>
#include <queue>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <string>
#include "types.hpp"
#include "logger.hpp"
#include "heartbeat.hpp"

using std::literals::string_literals::operator""s;

class Pelican : public rclcpp::Node {
    public:
        explicit Pelican();
        ~Pelican();

        int getID() const;
        std::string getModel() const;
        double getMass() const;
        possible_roles getRole() const;
        int getCurrentTerm() const;
        rclcpp::SubscriptionOptions getReentrantOptions() const;
        rclcpp::CallbackGroup::SharedPtr getReentrantGroup() const;

        static void signalHandler(int signum);
        static void setInstance(rclcpp::Node::SharedPtr instance);
        static std::shared_ptr<Pelican> getInstance();

        bool isLeader() const;
        bool isFollower() const;
        bool isCandidate() const;

        void resetElectionTimer();

        void commenceFollowerOperations();
        void setElectionStatus(int id);

        // TODO: move to private
        LoggerModule logger_;
        HeartbeatModule hb_core_;

    private: // Member functions

        void becomeLeader();
        void becomeFollower();
        void becomeCandidate();

        void setRole(possible_roles r);

        void parseModel();

        void printData(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) const;

        void requestVote();
        void serveVoteRequest(const comms::msg::RequestVoteRPC msg) const;
        void vote(int id_to_vote, double candidate_mass) const;
        void storeCandidacy(const comms::msg::Datapad::SharedPtr msg);
        void flushVotes();

        bool checkElectionTimedOut() const;
        void setElectionTimedOut();
        void unsetElectionTimedOut();

        void resetVotingWindow();

        void leaderElection();

        bool checkElectionCompleted() const;
        void setElectionCompleted();
        void unsetElectionCompleted();

        bool checkVotingCompleted() const;
        void setVotingCompleted();
        void unsetVotingCompleted();

        void ballotCheckingThread();
        void stopBallotThread();
        void startBallotThread();

        bool checkForExternalLeader();
        void clearElectionStatus();

        bool checkExternalLeaderElected() const;
        void setExternalLeaderElected();
        void unsetExternalLeaderElected();

        void setRandomElectionTimeout();
        void setRandomBallotWaittime();
        std::chrono::milliseconds getBallotWaitTime() const;

        bool checkLeaderElected() const;
        void setLeaderElected();
        void unsetLeaderElected();

        void setLeader(int id = -1);
        void setMass(double m);
        void increaseCurrentTerm();

        bool checkIsTerminated() const;
        void setIsTerminated();

        void prepareCommonCallbacks();

    private: // Attributes
        int id_;
        std::string model_;
        double mass_ {0.0};

        int leader_id_ {0};

        possible_roles role_ {tbd};

        int current_term_ {0};
        std::thread ballot_thread_;
        static std::weak_ptr<Pelican> instance_; // Weak pointer to the instance of the node

        // The subscription sets a QoS profile based on rmw_qos_profile_sensor_data.
        // This is needed because the default ROS 2 QoS profile for subscribers is
        // incompatible with the PX4 profile for publishers.
        rclcpp::QoS px4_qos_ {rclcpp::QoS(
            rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, 5),
            rmw_qos_profile_sensor_data
        )};
        rmw_qos_profile_t qos_profile_ {rmw_qos_profile_default};
        rclcpp::QoS standard_qos_ {
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile_), qos_profile_)
        };

        rclcpp::SubscriptionOptions reentrant_opt_ {rclcpp::SubscriptionOptions()};
        rclcpp::CallbackGroup::SharedPtr reentrant_group_;

        std::string leader_election_topic_ {"/fleet/leader_election"};
        rclcpp::Subscription<comms::msg::Datapad>::SharedPtr sub_to_leader_election_topic_;
        rclcpp::Publisher<comms::msg::Datapad>::SharedPtr pub_to_leader_election_topic_;

        std::string local_pos_topic_; // i.e. "/fmu/out/vehicle_local_position";
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
            sub_to_local_pos_topic_;

        std::string request_vote_rpc_topic_ {"/fleet/request_vote_rpc"};
        rclcpp::Publisher<comms::msg::RequestVoteRPC>::SharedPtr pub_to_request_vote_rpc_topic_;
        rclcpp::Subscription<comms::msg::RequestVoteRPC>::SharedPtr sub_to_request_vote_rpc_topic_;

        std::mt19937 random_engine_ {std::random_device {}()
        }; // mersenne_twister_engine seeded with random_device()
        std::uniform_int_distribution<> random_distribution_ {
            150, 300
        }; // inclusive; intended as milliseconds

        rclcpp::TimerBase::SharedPtr election_timer_;
        rclcpp::TimerBase::SharedPtr voting_timer_;

        // default memory ordering: std::memory_order_seq_cst -->
        // it guarantees sequential consistency (total global ordering) between all atomic
        // operations.
        std::atomic<bool> is_terminated_ {false};
        std::atomic<bool> leader_elected_ {false};
        std::atomic<bool> external_leader_elected_ {false};
        std::atomic<bool> election_completed_ {false
        }; // candidate (current node) has won an election
        std::atomic<bool> voting_completed_ {false
        }; // the appropriate amount of time after a vote to judge if all agents voted has passed

        std::condition_variable cv;

        mutable std::mutex votes_mutex_;              // to use with received_votes_
        mutable std::mutex election_completed_mutex_; // to use with election_completed_
        mutable std::mutex voting_completed_mutex_;   // to use with voting_completed_
        mutable std::mutex candidate_mutex_; // to use on the condition variable cv when the node is
                                             // a candidate and tries to win an election
        mutable std::mutex external_leader_mutex_; // to use with external_leader_elected_
        mutable std::mutex leader_mutex_;          // to use with leader_elected_
        mutable std::mutex terminated_mutex_;      // to use with is_terminated_

        // Time frame after which a candidacy will begin if no heartbeat from a leader has been
        // received. Raft uses randomized election timeouts to ensure that split votes are rare and
        // that they are resolved quickly
        std::chrono::milliseconds election_timeout_;
        // "election_timeout_" is thes same time window used by a follower to decide whether or not
        // to candidate as leader. using another variable only for clarity purposes
        std::chrono::milliseconds new_ballot_waittime_;
        std::chrono::milliseconds voting_max_time_ {100
        }; // tried using the same as it would be the period of a functioning leader's heartbeat;
           // works for now

        std::vector<comms::msg::Datapad::SharedPtr> received_votes_;
};

// Including templates definitions
#include "templates.tpp"

#endif
