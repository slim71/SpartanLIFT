#include "ElectionModule/election.hpp"
#include "PelicanModule/pelican.hpp"
#include "types.hpp"

/************************** Ctors/Dctors ***************************/
ElectionModule::ElectionModule() {
    this->node_ = nullptr;
    this->logger_ = nullptr;
}

ElectionModule::ElectionModule(Pelican* node) : node_(node), logger_ {nullptr} {}

ElectionModule::~ElectionModule() {
    this->stopService();

    // Cancel active timers
    cancelTimer(this->election_timer_);
    cancelTimer(this->voting_timer_);

    // Release mutexes
    std::lock_guard<std::mutex> lock_votes(this->votes_mutex_);
    std::lock_guard<std::mutex> lock_election_completed(this->election_completed_mutex_);
    std::lock_guard<std::mutex> lock_voting_completed(this->voting_completed_mutex_);
    std::lock_guard<std::mutex> lock_external_leader(this->external_leader_mutex_);
    std::lock_guard<std::mutex> lock_leader(this->leader_mutex_);
    std::lock_guard<std::mutex> lock_terminated(this->terminated_mutex_);
    std::lock_guard<std::mutex> lock_candidate(this->candidate_mutex_);

    // Clear shared resources
    this->received_votes_.clear();

    this->node_ = nullptr;
    this->logger_ = nullptr;
}

/************************** Setup methods **************************/
void ElectionModule::initSetup(LoggerModule* l) { // for followers and candidates
    this->logger_ = l;

    this->prepareTopics();
}

void ElectionModule::prepareTopics() {
    if (!this->node_) {
        throw MissingExternModule();
    }
    // If no error has been thrown, node_ is actually set and the rest can be executed

    /********************** Subscribrers **********************/
    // Subscribe to incoming votes from other agents
    if (!this->sub_to_leader_election_topic_) {
        this->sub_to_leader_election_topic_ =
            this->node_->create_subscription<comms::msg::Proposal>(
                this->leader_election_topic_, this->standard_qos_,
                std::bind(&ElectionModule::storeVotes, this, std::placeholders::_1),
                this->gatherReentrantOptions()
            );
    }

    /*********************** Publishers ***********************/
    // Setup a publisher to vote for an agent
    if (!this->pub_to_leader_election_topic_) {
        this->pub_to_leader_election_topic_ = this->node_->create_publisher<comms::msg::Proposal>(
            this->leader_election_topic_, this->standard_qos_
        );
    }
}

/********************** Core functionalities ***********************/
void ElectionModule::leaderElection() {
    // As for followers, even the candidates has stored all the votes
    this->votes_mutex_.lock();
    auto app_votes = this->received_votes_;
    this->votes_mutex_.unlock();

    if (!this->checkVotingCompleted() && app_votes.size() > 0) {
        // Sort votes based on the proposed leader
        std::sort(
            app_votes.begin(), app_votes.end(),
            [](const comms::msg::Proposal::SharedPtr& a, const comms::msg::Proposal::SharedPtr& b) {
                return a->proposed_leader < b->proposed_leader;
            }
        );

        std::vector<vote_count> ballot;

        // Create clusters with accumulated votes for each proposed leader
        for (auto it = std::cbegin(app_votes); it != std::cend(app_votes);
             /*iterator incremented inside*/) {
            vote_count candidate_support;
            candidate_support.candidate_id = (*it)->proposed_leader;
            candidate_support.total = std::count_if(
                it, std::cend(app_votes),
                [&](const comms::msg::Proposal::SharedPtr& v) {
                    return v->proposed_leader == (*it)->proposed_leader;
                }
            );
            ballot.push_back(candidate_support);

            auto last_it = it;
            ++last_it; // Move right away to the next element
            // Increment last_it until the last value of the cluster is found
            // (since votes are in order of proposed leader, this actually goes to the next one)
            while (last_it != std::cend(app_votes) &&
                   (*last_it)->proposed_leader == (*it)->proposed_leader) {
                ++last_it;
            }
            // Set the main iterator to the position after the last element of the cluster
            it = last_it;
        }

        this->sendLogDebug("Logging clusters..");
        for (const vote_count& canvote : ballot) {
            this->sendLogDebug(
                "Accumulated votes for candidate {}: {}", canvote.candidate_id, canvote.total
            );
        };

        auto cluster_for_this_node =
            std::find_if(ballot.begin(), ballot.end(), [this](const vote_count& v) {
                return v.candidate_id == this->gatherAgentID();
            });
        this->sendLogDebug(
            "Cluster for this node: candidate_id={} total={}",
            (*cluster_for_this_node).candidate_id, (*cluster_for_this_node).total
        );

        // If favorable votes >= network size, I'm the new leader
        if (!this->checkLeaderElected() && !this->checkExternalLeaderElected() &&
            (cluster_for_this_node->total >= (this->gatherNetworkSize() / 2 + 1))) {
            cancelTimer(this->election_timer_);
            this->sendLogInfo("Majority acquired!");
            this->setLeader(this->gatherAgentID());
            this->setLeaderElected();
            this->setVotingCompleted();
        } else {
            this->sendLogInfo("No majority yet");
        }

    } else {
        this->sendLogDebug("No votes yet or voting terminated");
    }
}

void ElectionModule::triggerVotes() {
    // Do not send request in case another leader has been already chosen
    if (this->confirmAgentIsCandidate() && this->checkExternalLeaderElected()) {
        this->sendLogWarning(
            "It appears another leader has been already chosen, so I won't trigger the vote request"
        );
        return;
    }

    this->sendLogInfo(
        "Triggering votes from the other agents during term {}", this->gatherCurrentTerm()
    );

    this->pub_to_request_vote_rpc_topic_ =
        this->node_->create_publisher<comms::msg::RequestVoteRPC>(
            this->request_vote_rpc_topic_, this->standard_qos_
        );

    comms::msg::RequestVoteRPC req;
    req.do_vote = true;
    req.solicitant_id = this->gatherAgentID();
    req.term_id = this->gatherCurrentTerm();

    this->pub_to_request_vote_rpc_topic_->publish(req);
}

void ElectionModule::serveVoteRequest(const comms::msg::RequestVoteRPC msg) const {
    this->sendLogInfo(
        "Serving vote request from candidate {} during term {}", msg.solicitant_id, msg.term_id
    );
    auto heavier = std::max_element(
        this->received_votes_.begin(), this->received_votes_.end(),
        [](comms::msg::Proposal::SharedPtr first, comms::msg::Proposal::SharedPtr second) {
            return first->candidate_mass > second->candidate_mass;
        }
    );
    this->vote((*heavier)->proposed_leader, (*heavier)->candidate_mass);
}

void ElectionModule::vote(int id_to_vote, double candidate_mass) const {
    // Do not even vote in case another leader has been already chosen
    if (this->confirmAgentIsCandidate() && this->checkExternalLeaderElected()) {
        this->sendLogWarning(
            "Not going to vote, because I'm a candidate but there's an external leader"
        );
        return;
    }

    auto msg = comms::msg::Proposal();
    msg.term_id = this->gatherCurrentTerm();
    msg.voter_id = this->gatherAgentID();
    msg.proposed_leader = id_to_vote;
    msg.candidate_mass = candidate_mass;

    this->sendLogInfo("Voting agent {} for term {}", id_to_vote, msg.term_id);
    this->pub_to_leader_election_topic_->publish(msg);
}

void ElectionModule::storeVotes(const comms::msg::Proposal::SharedPtr msg) {
    this->sendLogDebug(
        "Received vote| agent {} voted for agent {} (mass {}) during term {} ", msg->voter_id,
        msg->proposed_leader, msg->candidate_mass, msg->term_id
    );

    auto msg_term = msg->term_id;
    auto term = this->gatherCurrentTerm();

    // Ingore messages with past terms
    if (msg_term < term) {
        this->sendLogInfo("Ignoring candidacy referencing old term");
        return;
    }

    // Store vote
    this->votes_mutex_.lock();
    this->received_votes_.push_back(msg);
    this->votes_mutex_.unlock();

    auto role = this->gatherAgentRole();
    switch (role) {
        case follower:
            if (msg_term > term)
                this->signalNewTerm();
            break;
        case candidate:
            if (msg_term > term) { // another leader is elected, or new candidacy in progress
                this->sendLogWarning(
                    "Received vote with higher term (as candidate). Switching to follower"
                );
                this->signalNewTerm();
                this->signalTransitionToFollower();
            } else {
                this->leaderElection();
            }
            break;
        case leader:
            if (msg_term >= term) { // another leader is elected, or new candidacy in progress
                this->sendLogWarning(
                    "Received vote with equal or higher term (as leader). Switching to follower"
                );
                this->signalNewTerm();
                this->signalTransitionToFollower();
            }
            break;
        default:
            this->sendLogWarning(
                "Incorrect role (tbd) found when receiving a vote. Transitioning to follower"
            );
            this->signalTransitionToFollower();
    }
}

void ElectionModule::flushVotes() {
    std::lock_guard<std::mutex> lock(this->votes_mutex_);
    this->received_votes_.clear();
}

/************ Actions initiated from outside the module ************/
void ElectionModule::followerActions() {
    if (!this->node_) {
        throw MissingExternModule();
    }

    this->sendLogInfo("Setting a new election timeout");
    this->setRandomElectionTimeout();

    // Stay ready to serve a vote request from candidates
    this->sub_to_request_vote_rpc_topic_ =
        this->node_->create_subscription<comms::msg::RequestVoteRPC>(
            this->request_vote_rpc_topic_, this->standard_qos_,
            std::bind(&ElectionModule::serveVoteRequest, this, std::placeholders::_1),
            this->gatherReentrantOptions()
        );

    // Setup a timer to make sure a heartbeat is received. If not, switch to candidate
    this->election_timer_ = this->node_->create_wall_timer(
        this->election_timeout_,
        [this]() {
            cancelTimer(this->election_timer_);
            // 'if not needed, but just in case
            if (this->gatherAgentRole() == follower) {
                this->sendLogWarning("No heartbeat received within the 'election timeout' window; "
                                     "switching to candidate...");
                this->signalTransitionToCandidate();
            }
        },
        this->gatherReentrantGroup()
    );
}

void ElectionModule::candidateActions() {
    cancelTimer(this->election_timer_);
    resetSharedPointer(this->sub_to_request_vote_rpc_topic_); // unsubscribe from topic
    this->clearElectionStatus();

    while (!this->checkElectionCompleted()) {
        this->flushVotes(); // clean received votes

        this->sendLogInfo("Setting a new election timeout");
        this->setRandomElectionTimeout();
        this->election_timer_ = this->node_->create_wall_timer(
            this->election_timeout_,
            [this]() {
                cancelTimer(this->election_timer_);
                this->sendLogDebug(
                    "Election timeout elapsed for candidate agent during term {}",
                    this->gatherCurrentTerm()
                );
                this->setVotingCompleted();
            },
            this->gatherReentrantGroup()
        );

        this->signalNewTerm();
        this->sendLogInfo("New voting round for term {}", this->gatherCurrentTerm());

        // Self-vote and trigger votes from other agents
        this->vote(this->gatherAgentID(), this->gatherAgentMass());
        this->triggerVotes();

        // Wait for the ballot completion or the expiration of the election timeout
        while (!this->checkVotingCompleted()) {
            this->sendLogDebug("Ballot checking...");
            // Simulate some delay between checks
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        this->sendLogDebug("Ballot finished");

        // Stop if a leader is elected
        if (this->checkLeaderElected() || this->checkExternalLeaderElected()) {
            this->setElectionCompleted();
        }
    }

    // Do nothing, it's an action started before
    if (this->gatherAgentRole() == follower)
        return;

    // Handle external leader
    if (this->checkExternalLeaderElected()) {
        this->signalTransitionToFollower();
    }

    // Switch to leader in case of won election
    if (this->checkLeaderElected()) {
        this->signalTransitionToLeader();
    }
}

/************* Both from outside and inside the module *************/
void ElectionModule::stopService() {
    // this->candidate_mutex_.lock();
    // this->setIsBallotTerminated();
    // this->cv.notify_all(); // in this instance, either notify_one or notify_all should be the
    // same
    // // this->candidate_mutex_.unlock();
    // this->sendLogDebug("Notified through the condition variable while stopping the ballot
    // thread!");

    // if (this->ballot_thread_.joinable()) {
    //     this->sendLogDebug("Waiting on the ballot thread completion...");
    //     this->ballot_thread_.join();
    // }
}
