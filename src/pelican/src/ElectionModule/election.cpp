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

    // Clear shared resources
    this->received_votes_.clear();

    this->node_ = nullptr;
    this->logger_ = nullptr;
}

/************************** Setup methods **************************/
void ElectionModule::initSetup(LoggerModule* l) {
    this->logger_ = l;

    this->prepareTopics();
}

void ElectionModule::prepareTopics() {
    if (!this->node_) {
        throw MissingExternModule();
    }

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

    if (!this->checkVotingCompleted() && !app_votes.empty()) {
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
             /*iterator incremented inside the loop*/) {
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

        if (cluster_for_this_node == ballot.end()) {
            this->sendLogWarning("No votes accumulated!");
            return;
        }

        this->sendLogDebug(
            "Cluster for this node: candidate_id={} total={}",
            (*cluster_for_this_node).candidate_id, (*cluster_for_this_node).total
        );

        // If favorable votes >= network size/2, I'm the new leader
        if (!this->checkLeaderElected() && !this->checkExternalLeaderElected()) {
            if (cluster_for_this_node->total >= (this->gatherNetworkSize() / 2 + 1)) {
                cancelTimer(this->election_timer_);
                this->sendLogInfo("Majority acquired!");
                this->setLeader(this->gatherAgentID());
                this->setLeaderElected();
                this->setVotingCompleted();
            } else {
                this->sendLogInfo("No majority yet");
            }
        }

    } else {
        this->sendLogDebug("No votes yet or voting terminated");
    }
}

void ElectionModule::triggerVotes() {
    if (!this->node_) {
        throw MissingExternModule();
    }

    // Do not send request in case another leader has been already chosen
    if (this->confirmAgentIsCandidate() && this->checkExternalLeaderElected()) {
        this->sendLogWarning(
            "It appears another leader has been already chosen, so I won't trigger the vote request"
        );
        return;
    }

    this->sendLogInfo("Triggering votes from the other agents");

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

void ElectionModule::serveVoteRequest(const comms::msg::RequestVoteRPC msg) {
    if (msg.solicitant_id == this->gatherAgentID()) {
        this->sendLogDebug("Not serving my own vote request");
        return;
    }

    if (msg.term_id > this->gatherCurrentTerm()) {
        this->sendLogDebug("Aligning my term to the vote request");
        this->signalSetTerm(msg.term_id);
    }

    if (this->gatherAgentRole() == leader) {
        this->sendLogInfo("External vote request arrived; transitioning to follower...");
        this->signalSetTerm(msg.term_id);
        this->signalTransitionToFollower();
    }

    this->sendLogInfo(
        "Serving vote request from candidate {} for term {}", msg.solicitant_id, msg.term_id
    );
    // This returns the second iterator used, which here is '.end()' aka one past the
    // end of the sequence/vector
    auto heavier = std::max_element(
        this->received_votes_.begin(), this->received_votes_.end(),
        [](comms::msg::Proposal::SharedPtr first, comms::msg::Proposal::SharedPtr second) {
            return first->candidate_mass > second->candidate_mass;
        }
    );
    if ((heavier == this->received_votes_.end()) || ((*heavier)->proposed_leader <= 0)) {
        this->sendLogWarning("No valid ID to vote!");
        return;
    }
    this->sendLogDebug("Decided to vote for {}", (*heavier)->proposed_leader);
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

    comms::msg::Proposal msg = comms::msg::Proposal()
                                   .set__term_id(this->gatherCurrentTerm())
                                   .set__voter_id(this->gatherAgentID())
                                   .set__proposed_leader(id_to_vote)
                                   .set__candidate_mass(candidate_mass);

    this->sendLogInfo("Voting agent {}", id_to_vote);
    this->pub_to_leader_election_topic_->publish(msg);
}

void ElectionModule::storeVotes(const comms::msg::Proposal::SharedPtr msg) {
    this->sendLogDebug(
        "Received vote| agent {} voted for agent {} (mass {:.4f})", msg->voter_id,
        msg->proposed_leader, msg->candidate_mass
    );

    auto term = this->gatherCurrentTerm();

    // Ignore messages with past terms
    if (msg->term_id < term) {
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
            if (msg->term_id > term)
                this->signalSetTerm(msg->term_id);
            break;
        case candidate:
            // Another leader is elected, or new candidacy in progress
            if (msg->term_id > term) {
                this->sendLogWarning(
                    "Received vote with higher term (as candidate). Switching to follower"
                );
                this->signalSetTerm(msg->term_id);
                this->signalTransitionToFollower();
            } else {
                this->leaderElection();
            }
            break;
        case leader:
            // Another leader is elected, or new candidacy in progress
            if (msg->term_id >= term) {
                this->sendLogWarning(
                    "Received vote with equal or higher term (as leader). Switching to follower"
                );
                this->signalSetTerm(msg->term_id);
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
            // 'if' not needed, but just in case
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
    if (!this->node_) {
        throw MissingExternModule();
    }

    cancelTimer(this->election_timer_);
    resetSharedPointer(this->sub_to_request_vote_rpc_topic_); // Unsubscribe from topic
    this->clearElectionStatus();

    while (!this->checkElectionCompleted()) {
        this->flushVotes(); // Clear received votes

        this->sendLogInfo("Setting a new election timeout");
        this->setRandomElectionTimeout();
        this->election_timer_ = this->node_->create_wall_timer(
            this->election_timeout_,
            [this]() {
                cancelTimer(this->election_timer_);
                this->sendLogDebug("Election timeout elapsed for candidate agent");
                this->setVotingCompleted();
            },
            this->gatherReentrantGroup()
        );

        this->signalIncreaseTerm();
        this->sendLogInfo("New voting round");

        // Self-vote and trigger votes from other agents
        this->vote(this->gatherAgentID(), this->gatherAgentMass());
        this->triggerVotes();

        // Wait for the ballot completion or the expiration of the election timeout
        while (!this->checkVotingCompleted()) {
            this->sendLogDebug("Ballot checking...");
            // Simulate some delay between checks
            std::this_thread::sleep_for(std::chrono::milliseconds(constants::DELAY_MILLIS));
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
void ElectionModule::stopService() {}
