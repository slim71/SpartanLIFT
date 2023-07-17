#include "pelican.hpp"

void PelicanUnit::vote(int id_to_vote) {
    this->logInfo("Voting: {}", id_to_vote);

    auto msg = comms::msg::Datapad();
    msg.term_id = this->getCurrentTerm();
    msg.voter_id = this->getID();
    msg.proposed_leader = id_to_vote;

    this->pub_to_leader_selection_topic_->publish(msg);
}

void PelicanUnit::requestVote() {
    this->pub_to_request_vote_rpc_topic_ = this->create_publisher<comms::msg::RequestVoteRPC>(
                                                this->request_vote_rpc_topic_,
                                                this->standard_qos_
                                            );

    comms::msg::RequestVoteRPC req;
    req.do_vote = true;
    req.solicitant_id = this->getID();
    req.term_id = this->getCurrentTerm();

    this->pub_to_request_vote_rpc_topic_->publish(req);
}

void PelicanUnit::leaderElection() {
    // As for followers, even the candidates has stored all the votes
    this->votes_mutex_.lock();

    if (this->received_votes_.size() > 0) {
        std::sort(this->received_votes_.begin(), 
                this->received_votes_.end(), 
                [](const comms::msg::Datapad::SharedPtr &a, const comms::msg::Datapad::SharedPtr &b) {
                    return a->proposed_leader < b->proposed_leader;
                }
                );

        std::vector<vote_count> ballot;

        for(auto it = std::cbegin(this->received_votes_); it != std::cend(this->received_votes_); ) {  
            vote_count candidate_support;
            candidate_support.candidate_id = (*it)->proposed_leader;
            candidate_support.total = std::count_if(it, 
                                                    std::cend(this->received_votes_), 
                                                    [&](const comms::msg::Datapad::SharedPtr &v) {
                                                        return v->proposed_leader == (*it)->proposed_leader;
                                                    }
                                                    );
            ballot.push_back(candidate_support);

            // Increment the iterator until the last value of the cluster is found
            for(auto last = (*it)->proposed_leader; (*++it)->proposed_leader == last;);
        }

        this->votes_mutex_.unlock();

        std::sort(ballot.begin(),
                ballot.end(),
                [](vote_count a, vote_count b){
                    return a.total < b.total;
                }
                );
        auto item = std::find_if(ballot.begin(),
                                ballot.end(),
                                [this](const vote_count &v){
                                    return v.candidate_id == this->getID();
                                }
                                );
        
        // If item is the last element of the ballot vector, either this candidate has won the election or there's a tie
        if((*item).total == (*ballot.cend()).total) {
            if((*item).total != (*(ballot.cend()-1)).total) { // this candidate has won
                // I've been chosen!
                this->setElectionCompleted();
                this->setLeader(this->getID());
                this->becomeLeader();
                this->sendHeartbeat(); // to be sure one is sent now; TODO: maybe move to leader? needed?
                return;
            };
        };
        // Otherwise, let the winning candidate send its heartbeat as leader confirmation
    }

    this->votes_mutex_.unlock();

    // Each candidate will time out and start a new election by incrementing its term and initiating another round.
    // Each candidate restarts its randomized election timeout at the start of an election, and it waits 
    // for that timeout to elapse before starting the next election.

    this->resetVotingWindow();
}

void PelicanUnit::setLeader(int id = -1) {
    this->setLeaderElected();

    // TODO: sync with other nodes first?
    if (id == -1) {
        this->leader_id_ = this->getID();
    } else {
        this->leader_id_ = id;
    }
}

void PelicanUnit::serveVoteRequest(const comms::msg::RequestVoteRPC msg) {
    auto heavier = std::max_element(this->received_votes_.begin(),
                                    this->received_votes_.end(),
                                    [](comms::msg::Datapad::SharedPtr first, comms::msg::Datapad::SharedPtr second) {
                                        return first->candidate_mass > second->candidate_mass;
                                    }
                                    );
    this->vote((*heavier)->proposed_leader);
}

bool PelicanUnit::checkForExternalLeader() {
    // If the (external) leader’s term is at least as large as the candidate’s current term, 
    // then the candidate recognizes the leader as legitimate and returns to follower state

    if(!this->checkExternalLeaderElected()) {
        return false;
    };

    this->hbs_mutex_.lock();
    if (this->received_hbs_.size() == 0) {
        this->hbs_mutex_.unlock();    
        return true; // standard behavior: boolean is more important than the heartbeat
    }

    auto last_hb_received = this->received_hbs_.back();
    this->hbs_mutex_.unlock();

    if (last_hb_received.term >= this->getCurrentTerm()) {
        return true;
    } else {
        this->unsetExternalLeaderElected(); // reject external leader and continue as no heartbeat arrived
        return false;
    }
}

void PelicanUnit::resetVotingWindow() {
    this->unsetVotingCompleted();
    this->setRandomBallotWaittime();
}

void PelicanUnit::storeCandidacy(const comms::msg::Datapad::SharedPtr msg) {
    std::cout << "\n\n";
    std::cout << "AGENT " << this->getID() << " RECEIVED DATAPAD DATA"   << std::endl;
    std::cout << "============================="   << std::endl;
    std::cout << "term_id: " << msg->term_id << std::endl;
    std::cout << "voter_id: " << msg->voter_id << std::endl;
    std::cout << "proposed_leader: " << msg->proposed_leader << std::endl;
    std::cout << "candidate_mass: " << msg->candidate_mass << std::endl;
    std::cout << "\n\n";

    this->votes_mutex_.lock();
    this->received_votes_.push_back(msg);
    this->votes_mutex_.unlock();

    // Votes received are supposedly from followers, so if no more votes come within a time frame, the elections is finished
    if(this->getRole() == candidate) {
        if(this->voting_timer != nullptr) { // Make the timer restart
            this->voting_timer->reset();
        } else{
            this->voting_timer = this->create_wall_timer(this->voting_max_time_, 
                                                         std::bind(&PelicanUnit::setElectionCompleted, this),
                                                         this->reentrant_group_);
        };
    };

}

void PelicanUnit::flushVotes() {
    std::lock_guard<std::mutex> lock(this->votes_mutex_);
    this->received_votes_.clear();
}

void PelicanUnit::ballotCheckingThread() {
    // Continuously check for timeout or interrupt signal
    while (!this->checkVotingCompleted() && !this->checkForExternalLeader() && !this->is_terminated_) {
        this->logDebug("Ballot checking...");
        // Simulate some delay between checks
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    this->logDebug("Ballot finished");

    // Notify the first thread to stop waiting
    cv.notify_all(); // in this instance, either notify_one or notify_all should be the same
}
