#include "election.hpp"
#include "pelican.hpp"
#include "logger.hpp"

ElectionModule::ElectionModule(Pelican* node) : node_(node) {

}

ElectionModule::~ElectionModule() {
    this->sendLogDebug("Trying to kill the ballot thread");
    this->stopBallotThread();

    // Cancel active timers
    cancelTimer(this->election_timer_);
    cancelTimer(this->voting_timer_);

    // Unsubscribe from topics
    this->sub_to_leader_election_topic_.reset();
    this->sub_to_request_vote_rpc_topic_.reset();

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

}

void ElectionModule::vote(int id_to_vote, double candidate_mass) const {
    // Do not even vote in case another leader has been already chosen
    if (this->node_->isCandidate() && this->checkExternalLeaderElected()) {
        return;
    }

    this->sendLogInfo("Voting: {}", id_to_vote);

    auto msg = comms::msg::Datapad();
    msg.term_id = this->node_->getCurrentTerm();
    msg.voter_id = this->node_->getID();
    msg.proposed_leader = id_to_vote;
    msg.candidate_mass = candidate_mass;

    this->pub_to_leader_election_topic_->publish(msg);
}

void ElectionModule::requestVote() {
    // Do not serve request in case another leader has been already chosen
    if (this->node_->isCandidate() && this->checkExternalLeaderElected()) {
        return;
    }

    this->sendLogDebug("Requesting votes to other agents...");

    this->pub_to_request_vote_rpc_topic_ = this->node_->create_publisher<comms::msg::RequestVoteRPC>(
        this->request_vote_rpc_topic_, this->standard_qos_
    );

    comms::msg::RequestVoteRPC req;
    req.do_vote = true;
    req.solicitant_id = this->node_->getID();
    req.term_id = this->node_->getCurrentTerm();

    this->pub_to_request_vote_rpc_topic_->publish(req);
}

void ElectionModule::leaderElection() {
    // As for followers, even the candidates has stored all the votes
    this->votes_mutex_.lock();

    if (this->received_votes_.size() > 0) {
        std::sort(
            this->received_votes_.begin(), this->received_votes_.end(),
            [](const comms::msg::Datapad::SharedPtr& a, const comms::msg::Datapad::SharedPtr& b) {
                return a->proposed_leader < b->proposed_leader;
            }
        );

        this->sendLogDebug("Logging recorded votes..");
        for (const comms::msg::Datapad::SharedPtr& recvote : this->received_votes_) {
            this->sendLogDebug( 
                "agent {} voted for {} (mass {}) during term {} ", recvote->voter_id,
                recvote->proposed_leader, recvote->candidate_mass, recvote->term_id
            );
        };

        std::vector<vote_count> ballot;

        for (auto it = std::cbegin(this->received_votes_); it != std::cend(this->received_votes_);
             /*iterator incremented inside*/) {
            vote_count candidate_support;
            candidate_support.candidate_id = (*it)->proposed_leader;
            candidate_support.total = std::count_if(
                it, std::cend(this->received_votes_),
                [&](const comms::msg::Datapad::SharedPtr& v) {
                    return v->proposed_leader == (*it)->proposed_leader;
                }
            );
            ballot.push_back(candidate_support);

            auto last_it = it;
            ++last_it; // Move right away to the next element
            // Increment last_it until the last value of the cluster is found
            while (last_it != std::cend(this->received_votes_) &&
                   (*last_it)->proposed_leader == (*it)->proposed_leader) {
                ++last_it;
            }
            // Set the main iterator to the position after the last element of the cluster
            it = last_it;
        }

        this->votes_mutex_.unlock();

        std::sort(ballot.begin(), ballot.end(), [](vote_count a, vote_count b) {
            return a.total < b.total;
        });

        this->sendLogDebug("Logging clusters..");
        for (const vote_count& canvote : ballot) {
            this->sendLogDebug( 
                "Accumulated votes for candidate {}: {}", canvote.candidate_id, canvote.total
            );
        };

        auto cluster_for_this_node =
            std::find_if(ballot.begin(), ballot.end(), [this](const vote_count& v) {
                return v.candidate_id == this->node_->getID();
            });
        this->sendLogDebug(
            "cluster for this node: candidate_id={} total={}",
            (*cluster_for_this_node).candidate_id, (*cluster_for_this_node).total
        );

        if (ballot.size() > 0) { // back on empty vector has undefined behavior

            // If cluster_for_this_node is the last element of the ballot vector, either this
            // candidate has won the election or there's a tie
            if ((*cluster_for_this_node).total == ballot.back().total) {
                this->sendLogDebug("Victory or not?");

                if ((ballot.size() == 1) ||
                    (*cluster_for_this_node).total !=
                        (ballot.end()[-2]).total) { // this candidate has won
                    this->sendLogDebug("I've won!");
                    // I've been chosen!
                    this->setElectionCompleted();
                    this->setLeader(this->node_->getID());
                    this->node_->commenceLeaderOperations();
                    return;
                }

                this->sendLogDebug("Tie or timeout");
            }
        }
        // Otherwise, let the winning candidate send its heartbeat as leader confirmation
    } else {
        this->votes_mutex_.unlock();
    }

    // Each candidate will time out and start a new election by incrementing its term and initiating
    // another round. Each candidate restarts its randomized election timeout at the start of an
    // election, and it waits for that timeout to elapse before starting the next election.

    this->resetVotingWindow();
}

void ElectionModule::serveVoteRequest(const comms::msg::RequestVoteRPC msg) const {
    auto heavier = std::max_element(
        this->received_votes_.begin(), this->received_votes_.end(),
        [](comms::msg::Datapad::SharedPtr first, comms::msg::Datapad::SharedPtr second) {
            return first->candidate_mass > second->candidate_mass;
        }
    );
    this->vote((*heavier)->proposed_leader, (*heavier)->candidate_mass);
}

bool ElectionModule::checkForExternalLeader() {
    // If the (external) leader’s term is at least as large as the candidate’s current term,
    // then the candidate recognizes the leader as legitimate and returns to follower state

    if (!this->checkExternalLeaderElected()) {
        return false;
    }

    if (this->node_->requestNumberOfHbs() == 0) {
        // Standard behavior: boolean is more important than the heartbeat
        // This is equal to "no heartbeat received YET, but external leader is there"
        return true;
    }

    auto last_hb_received = this->node_->requestLastHb();

    if (last_hb_received.term >= this->node_->getCurrentTerm()) {
        return true;
    } else {
        // reject external leader and continue as no heartbeat arrived
        this->sendLogDebug("Most recent heartbeat has old term");
        this->clearElectionStatus();
        return false;
    }
}

void ElectionModule::resetVotingWindow() {
    this->unsetVotingCompleted();
    this->setRandomBallotWaittime();
}

void ElectionModule::storeCandidacy(const comms::msg::Datapad::SharedPtr msg) {
    std::cout << "\n\n";
    std::cout << "AGENT " << this->node_->getID() << " RECEIVED DATAPAD DATA" << std::endl;
    std::cout << "=============================" << std::endl;
    std::cout << "term_id: " << msg->term_id << std::endl;
    std::cout << "voter_id: " << msg->voter_id << std::endl;
    std::cout << "proposed_leader: " << msg->proposed_leader << std::endl;
    std::cout << "candidate_mass: " << msg->candidate_mass << std::endl;
    std::cout << "\n\n";

    this->sendLogDebug(
        "Received vote| agent {} voted for {} (mass {}) during term {} ", msg->voter_id,
        msg->proposed_leader, msg->candidate_mass, msg->term_id
    );

    this->votes_mutex_.lock();
    this->received_votes_.push_back(msg);
    this->votes_mutex_.unlock();

    // Votes received are supposedly from followers, so if no more votes come within a time frame,
    // the elections is finished
    if (this->node_->getRole() == candidate) {
        if (this->voting_timer_ != nullptr) { // Make the timer restart
            this->voting_timer_->reset();
        } else {
            this->voting_timer_ = this->node_->create_wall_timer(
                this->voting_max_time_, std::bind(&ElectionModule::setVotingCompleted, this),
                this->node_->getReentrantGroup()
            );
        }
    }
}

void ElectionModule::flushVotes() {
    std::lock_guard<std::mutex> lock(this->votes_mutex_);
    this->received_votes_.clear();
}

void ElectionModule::ballotCheckingThread() {
    // Continuously check for timeout or interrupt signal
    while (!this->checkVotingCompleted() && !this->checkForExternalLeader() &&
           !this->checkIsTerminated()) {
        this->sendLogDebug("Ballot checking...");
        // Simulate some delay between checks
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    this->sendLogInfo("Ballot finished");

    // Notify the main thread to stop waiting
    this->cv.notify_all(); // in this instance, either notify_one or notify_all should be the same
}

void ElectionModule::clearElectionStatus() {
    this->unsetExternalLeaderElected();
    this->unsetLeaderElected();
    this->setLeader();
}

void ElectionModule::setElectionStatus(int id) {
    this->setExternalLeaderElected();
    this->setLeaderElected();
    this->setLeader(id);
}

void ElectionModule::resetElectionTimer() {
    // Reset the election_timer_, used to be sure there's a leader, to election_timeout
    resetTimer(this->election_timer_); 
    this->sendLogDebug(
        "After resetting, timer is {} ms",this->election_timer_->time_until_trigger().count() / 10
    );
}

void ElectionModule::startBallotThread() {
    if (!this->ballot_thread_.joinable()) {
        this->ballot_thread_ = std::thread(&ElectionModule::ballotCheckingThread, this);
    }
}

void ElectionModule::stopBallotThread() {
    if (this->ballot_thread_.joinable()) {
        this->setIsTerminated();
        this->ballot_thread_.join();
    }
}

void ElectionModule::initSetup(LoggerModule* l) { // for followers and candidates
    this->logger_ = l;

    this->prepareTopics();

}

void ElectionModule::prepareTopics() {

    /******************* Subscribrers ****************************/
    if (!this->sub_to_leader_election_topic_) {
        this->sub_to_leader_election_topic_ = this->node_->create_subscription<comms::msg::Datapad>(
            this->leader_election_topic_, this->standard_qos_,
            std::bind(&ElectionModule::storeCandidacy, this, std::placeholders::_1),
            this->node_->getReentrantOptions()
        );
    }

    /******************* Publishers ****************************/
    if (!this->pub_to_leader_election_topic_) {
        this->pub_to_leader_election_topic_ = this->node_->create_publisher<comms::msg::Datapad>(
            this->leader_election_topic_, this->standard_qos_
        );
    }

}

void ElectionModule::signalStopBallotThread() {
    this->stopBallotThread();
}

void ElectionModule::followerActions() {
    this->setRandomElectionTimeout();

    this->sub_to_request_vote_rpc_topic_ = this->node_->create_subscription<comms::msg::RequestVoteRPC>(
        this->request_vote_rpc_topic_, this->standard_qos_,
        std::bind(&ElectionModule::serveVoteRequest, this, std::placeholders::_1), 
        this->node_->getReentrantOptions()
    );

    this->election_timer_ = this->node_->create_wall_timer(
        this->election_timeout_,
        [this]() {
            cancelTimer(this->election_timer_);
            if (this->node_->getRole() == follower) {
                this->sendLogWarning("No heartbeat received within the 'election_timeout' window; "
                                 "switching to candidate...");
                this->node_->commenceCandidateOperations(); // transition to Candidate state
            }
        },
        this->node_->getReentrantGroup()
    );
}

void ElectionModule::candidateActions() {
    // the candidate repeats this until:
    // (a) it wins the election                         --> election_completed_ = true
    // (b) another server establishes itself as leader  --> election_completed_ = false +
    // external_leader_elected = true (?) (c) a period of time goes by with no winner      -->
    // nothing signaling this, just restart from a newer term_id
    while (!this->checkElectionCompleted()) {
        // For the first execution the init phase has been done before the while loop

        this->flushVotes();

        this->sendLogDebug("Going to sleep for {} ms...", this->getBallotWaitTime().count());
        std::this_thread::sleep_for(this->getBallotWaitTime());

        this->node_->increaseCurrentTerm();
        this->sendLogInfo("New voting round for term {}", this->node_->getCurrentTerm());

        this->vote(this->node_->getID(), this->node_->getMass());
        this->requestVote();

        this->startBallotThread();

        // Wait until voting is completed (aka no more votes are registered in a time frame) or
        // timed-out, which is checked by another thread
        std::unique_lock<std::mutex> lock(this->candidate_mutex_);
        this->cv.wait(lock, [this]() {
            return (this->checkVotingCompleted() || this->checkForExternalLeader());
        });
        this->sendLogDebug("Main thread free from ballot lock");

        if (this->checkForExternalLeader()) {
            this->sendLogInfo("External leader elected");
            this->setElectionCompleted();
            this->node_->commenceFollowerOperations();
            continue; // or break, should be the same since the condition is set
        }

        if (this->checkVotingCompleted()) {
            this->sendLogInfo(
                "Voting completed for term {}. Checking if I'm the winner...",
                this->node_->getCurrentTerm()
            );
            this->leaderElection(
            ); // if this node is not the leader, election_completed_ is not true
        }

        // No other possible reasons to exit the condition_variable wait
    };
}

void ElectionModule::resetSubscriptions() {
    // This is not needed by leaders, since Raft guarantees safety;
    // it's just an additional check to avoid multiple leaders
    resetSharedPointer(this->sub_to_leader_election_topic_);
    resetSharedPointer(this->sub_to_request_vote_rpc_topic_);
}

void ElectionModule::prepareForCandidateActions() {
    cancelTimer(this->election_timer_);
    resetSharedPointer(this->sub_to_request_vote_rpc_topic_); // unsubscribe from topic

    this->clearElectionStatus();
    this->unsetVotingCompleted();
    this->unsetElectionCompleted();

    // init of new_ballot_waittime_
    this->setRandomBallotWaittime();
}