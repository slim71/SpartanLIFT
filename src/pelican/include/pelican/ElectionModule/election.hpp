#ifndef _ELECTION_HPP_
#define _ELECTION_HPP_

#include "LoggerModule/logger.hpp"
#include "types.hpp"
#include "utilities.hpp"

class Pelican;

class ElectionModule {
    public:
        // Ctors/Dctors
        explicit ElectionModule();
        explicit ElectionModule(Pelican*);
        ~ElectionModule();

        // Setup methods
        void initSetup(LoggerModule*);
        void prepareTopics();

        int getLeaderID();

        // Actions initiated from outside the module
        void resetElectionTimer();
        void resetSubscriptions();
        void setElectionStatus(int id);
        void followerActions();
        void candidateActions();

        // Both from outside and inside the module
        void stopService();
        void flushVotes();

    private: // Member functions
        template<typename... Args> void sendLogInfo(std::string, Args...) const;
        template<typename... Args> void sendLogDebug(std::string, Args...) const;
        template<typename... Args> void sendLogWarning(std::string, Args...) const;
        template<typename... Args> void sendLogError(std::string, Args...) const;

        // Core functionalities
        void leaderElection();
        void triggerVotes();
        void serveVoteRequest(const comms::msg::RequestVoteRPC msg) const;
        void vote(int id_to_vote, double candidate_mass) const;
        void storeVotes(const comms::msg::Proposal::SharedPtr msg);

        // External communications
        unsigned int gatherAgentID() const;
        double gatherAgentMass() const;
        possible_roles gatherAgentRole() const;
        unsigned int gatherCurrentTerm() const;
        int gatherNumberOfHbs() const;
        heartbeat gatherLastHb() const;
        int gatherNetworkSize() const;
        rclcpp::CallbackGroup::SharedPtr gatherReentrantGroup() const;
        rclcpp::SubscriptionOptions gatherReentrantOptions() const;
        bool confirmAgentIsCandidate() const;
        void signalIncreaseTerm() const;
        void signalSetTerm(uint64_t) const;
        void signalTransitionToLeader() const;
        void signalTransitionToCandidate() const;
        void signalTransitionToFollower() const;

        bool checkElectionCompleted() const;
        void setElectionCompleted();
        void unsetElectionCompleted();

        bool checkVotingCompleted() const;
        void setVotingCompleted();
        void unsetVotingCompleted();

        void setRandomElectionTimeout();

        void clearElectionStatus();

        bool checkForExternalLeader();
        bool checkExternalLeaderElected() const;
        void setExternalLeaderElected();
        void unsetExternalLeaderElected();
        bool checkLeaderElected() const;
        void setLeaderElected();
        void unsetLeaderElected();
        void setLeader(int id = 0);

    private: // Attributes
        Pelican* node_;
        LoggerModule* logger_;

        int leader_id_ {0};

        std::thread ballot_thread_;

        rclcpp::QoS standard_qos_ {rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default), rmw_qos_profile_default
        )};

        std::string leader_election_topic_ {"/fleet/leader_election"};
        rclcpp::Subscription<comms::msg::Proposal>::SharedPtr sub_to_leader_election_topic_;
        rclcpp::Publisher<comms::msg::Proposal>::SharedPtr pub_to_leader_election_topic_;

        std::string request_vote_rpc_topic_ {"/fleet/request_vote_rpc"};
        rclcpp::Publisher<comms::msg::RequestVoteRPC>::SharedPtr pub_to_request_vote_rpc_topic_;
        rclcpp::Subscription<comms::msg::RequestVoteRPC>::SharedPtr sub_to_request_vote_rpc_topic_;

        rclcpp::TimerBase::SharedPtr election_timer_;
        rclcpp::TimerBase::SharedPtr voting_timer_;

        // candidate (current node) has won an election
        std::atomic<bool> election_completed_ {false};
        // the appropriate amount of time after a vote to judge if all agents voted has passed
        std::atomic<bool> voting_completed_ {false};
        // default memory ordering: std::memory_order_seq_cst -->
        // it guarantees sequential consistency (total global ordering) between all atomic
        // operations.
        std::atomic<bool> is_terminated_ {false};
        std::atomic<bool> leader_elected_ {false};
        std::atomic<bool> external_leader_elected_ {false};

        std::condition_variable cv;

        mutable std::mutex votes_mutex_;              // to use with received_votes_
        mutable std::mutex election_completed_mutex_; // to use with election_completed_
        mutable std::mutex voting_completed_mutex_;   // to use with voting_completed_
        mutable std::mutex external_leader_mutex_;    // to use with external_leader_elected_
        mutable std::mutex leader_mutex_;             // to use with leader_elected_
        mutable std::mutex terminated_mutex_;         // to use with is_terminated_
        mutable std::mutex candidate_mutex_; // to use on the condition variable cv when the node is
                                             // a candidate and tries to win an election

        // Time frame after which a candidacy will begin if no heartbeat from a leader has been
        // received. Raft uses randomized election timeouts to ensure that split votes are rare and
        // that they are resolved quickly
        std::chrono::milliseconds election_timeout_;
        // "election_timeout_" is the same time window used by a follower to decide whether or not
        // to candidate as leader. using another variable only for clarity purposes
        std::chrono::milliseconds new_ballot_waittime_;
        // tried using the same as it would be the period of a functioning leader's heartbeat;
        // works for now
        std::chrono::milliseconds voting_max_time_ {100};

        std::vector<comms::msg::Proposal::SharedPtr> received_votes_;

        // mersenne_twister_engine seeded with random_device()
        std::mt19937 random_engine_ {std::random_device {}()};
        // inclusive; intended as milliseconds
        std::uniform_int_distribution<> random_distribution_ {150, 300};
};

#include "election_templates.tpp"

#endif
