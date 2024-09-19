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
    // Reset statuses, to achieve end of operations
    this->setElectionCompleted();
    this->setVotingCompleted();

    // Cancel active timers
    cancelTimer(this->election_timer_);
    resetSharedPointer(this->election_timer_);

    // Clear shared pointers for subscriptions and publishers
    resetSharedPointer(sub_to_leader_election_topic_);
    resetSharedPointer(pub_to_leader_election_topic_);
    resetSharedPointer(sub_to_request_vote_rpc_topic_);
    resetSharedPointer(pub_to_request_vote_rpc_topic_);

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

    if (this->isVotingCompleted() || app_votes.empty()) {
        this->sendLogDebug(
            "No votes yet ({}) or voting terminated ({}): stopping leader election",
            app_votes.empty(), this->isVotingCompleted()
        );
        return;
    }

    // Sort votes based on the proposed leader
    std::unordered_map<int, int> vote_counts; // Map with pairs: proposed leader - vote count

    // Count votes
    for (const auto& vote : app_votes) {
        vote_counts[vote->proposed_leader]++;
    }
    this->sendLogDebug("Logging accumulated votes...");
    for (const auto& [candidate_id, total] : vote_counts) {
        this->sendLogDebug("Accumulated votes for candidate {}: {}", candidate_id, total);
    }

    // Find the cluster (votes) for this node
    unsigned int my_id = this->gatherAgentID();
    auto cluster_for_this_node = vote_counts.find(my_id);
    if (cluster_for_this_node == vote_counts.end()) {
        this->sendLogWarning("No votes accumulated for me!");
        return;
    }

    this->sendLogDebug(
        "Cluster for this node: candidate_id={} total={}", cluster_for_this_node->first,
        cluster_for_this_node->second
    );
    // If favorable votes >= network size/2, I'm the new leader
    if (!this->isLeaderElected() && !this->isExternalLeaderElected()) {
        if (cluster_for_this_node->second >= (this->gatherNetworkSize() / 2 + 1)) {
            cancelTimer(this->election_timer_);
            this->sendLogInfo("Majority acquired!");
            this->setVotingCompleted();
            this->setElectionCompleted();
            this->setLeader(this->gatherAgentID());
            this->setLeaderElected();
        } else {
            this->sendLogInfo("No majority yet");
        }
    }
}

void ElectionModule::triggerVotes() {
    if (!this->node_) {
        throw MissingExternModule();
    }

    // Do not send request in case another leader has been already chosen
    if (!this->confirmAgentIsCandidate() || this->isExternalLeaderElected()) {
        this->sendLogWarning(
            "It appears another leader has been already chosen (agent {}), so I won't trigger the "
            "vote request",
            this->getLeaderID()
        );
        this->setElectionCompleted(); // Stop retrying
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
    // Ensure the requesting agent is already been discovered
    this->signalStoreAttendance(msg.solicitant_id);

    if (msg.term_id > this->gatherCurrentTerm()) {
        this->sendLogDebug("Aligning my term to the vote request");
        this->signalSetTerm(msg.term_id);
    }

    if ((this->gatherAgentRole() != follower) && (msg.term_id > this->gatherCurrentTerm())) {
        this->sendLogInfo("External vote request arrived; transitioning to follower...");
        this->signalSetTerm(msg.term_id);
        this->setElectionStatus(0);
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
    this->setRandomElectionTimeout();
    this->vote((*heavier)->proposed_leader, (*heavier)->candidate_mass);
}

void ElectionModule::vote(int id_to_vote, double candidate_mass) const {
    // Do not even vote in case another leader has been already chosen
    if (this->confirmAgentIsCandidate() && this->isExternalLeaderElected()) {
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
    // Ensure the requesting agent is already been discovered
    this->signalStoreAttendance(msg->voter_id);

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
            this->sendLogDebug("Setting term from the received vote: {}", msg->term_id);
            this->signalSetTerm(msg->term_id);
            break;
        case candidate:
            // Another leader is elected, or new candidacy in progress
            if (msg->term_id > term) {
                this->sendLogWarning(
                    "Received vote from agent {} for agent {} with higher term. Switching to "
                    "follower",
                    msg->voter_id, msg->proposed_leader
                );
                this->setElectionStatus(0);
                this->signalSetTerm(msg->term_id);
                this->signalTransitionToFollower();
            } else {
                this->leaderElection();
            }
            break;
        case leader:
            // Another leader is elected, or new candidacy in progress
            this->sendLogWarning("Received vote with equal or higher term. Switching to follower");
            this->signalSetTerm(msg->term_id);
            this->setElectionStatus(0);
            this->signalTransitionToFollower();
            break;
        default:
            this->sendLogWarning(
                "Undefined role found when receiving a vote. Transitioning to follower"
            );
            this->setElectionStatus(0);
            this->signalTransitionToFollower();
    }
}

void ElectionModule::flushVotes() {
    std::lock_guard lock(this->votes_mutex_);
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
    this->clearElectionStatus();

    while (!this->isElectionCompleted()) {
        this->flushVotes(); // Clear received votes

        this->signalIncreaseTerm();
        this->sendLogInfo("Starting new voting round during term {}", this->gatherCurrentTerm());

        this->sendLogInfo("Setting a new election timeout");
        this->setRandomElectionTimeout();
        this->election_timer_ = this->node_->create_wall_timer(
            this->election_timeout_,
            [this]() {
                cancelTimer(this->election_timer_);
                this->sendLogDebug("Election timeout elapsed for candidate agent");
                this->setVotingCompleted();
            },
            // No need for an exclusive group, since it cancels itself
            this->gatherReentrantGroup()
        );

        // Self-vote and trigger votes from other agents
        this->clearElectionStatus();
        this->vote(this->gatherAgentID(), this->gatherAgentMass());
        this->triggerVotes();

        // Wait for the ballot completion or the expiration of the election timeout
        while (!this->isVotingCompleted()) {
            this->sendLogDebug("Ballot checking...");
            // Simulate some delay between checks
            std::this_thread::sleep_for(std::chrono::milliseconds(constants::DELAY_MILLIS));
        }
        this->sendLogDebug("Ballot finished");

        // Stop if a leader is elected
        if (this->isLeaderElected() || this->isExternalLeaderElected()) {
            this->setElectionCompleted();
            break;
        }
    }

    // Do nothing, it's an action started before
    if (this->gatherAgentRole() != candidate)
        return;

    // Handle external leader
    if (this->isExternalLeaderElected()) {
        this->signalTransitionToFollower();
    }

    // Switch to leader in case of won election
    if (this->isLeaderElected()) {
        this->signalTransitionToLeader();
    }
}

/************* Both from outside and inside the module *************/
void ElectionModule::stopService() {
    this->sendLogWarning("Stopping Election module!");

    // Reset statuses, to achieve end of operations
    this->setElectionStatus(0);

    // Cancel active timers
    cancelTimer(this->election_timer_);
    resetSharedPointer(this->election_timer_);

    // Clear shared pointers for subscriptions and publishers
    resetSharedPointer(this->sub_to_leader_election_topic_);
    resetSharedPointer(this->pub_to_leader_election_topic_);
    resetSharedPointer(this->sub_to_request_vote_rpc_topic_);
    resetSharedPointer(this->pub_to_request_vote_rpc_topic_);
}
