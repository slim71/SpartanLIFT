#include "pelican.hpp"

void PelicanUnit::vote(int id_to_vote) {
    RCLCPP_INFO(get_logger(), "Voting: %d", id_to_vote);

    auto msg = comms::msg::Datapad();

    // int64 term_id
    // int64 voter_id
    // int64 proposed_leader
    msg.term_id = this->current_term_;
    msg.voter_id = this->id_;
    msg.proposed_leader = id_to_vote;

    this->pub_to_leader_selection_topic_->publish(msg);
}

void PelicanUnit::requestVote() {
    if (this->pub_to_request_vote_rpc_topic_ == nullptr) {
        this->pub_to_request_vote_rpc_topic_ = this->create_publisher<comms::msg::RequestVoteRPC>(
                                                    this->request_vote_rpc_topic_, 10
                                                ); // TODO: 10 (or better value) in constant
    };

    comms::msg::RequestVoteRPC req;
    req.do_vote = true;
    req.solicitant_id = this->id_;
    req.term_id = this->current_term_;

    this->pub_to_request_vote_rpc_topic_->publish(req);
}

void PelicanUnit::leaderElection() {
    // As for followers, even the candidates has stored all the votes
    // TODO: case of no votes inserted
    
    // Copy because I don't want to modify the original vector TODO: needed?
    std::vector<comms::msg::Datapad::SharedPtr> votes_copy(received_votes);
    std::sort(votes_copy.begin(), 
              votes_copy.end(), 
              [](const comms::msg::Datapad::SharedPtr &a, const comms::msg::Datapad::SharedPtr &b) {
                  return a->proposed_leader < b->proposed_leader;
              }
            );

    std::vector<vote_count> ballot;

    for(auto it = std::cbegin(votes_copy); it != std::cend(votes_copy); ) {  
        vote_count candidate_support;
        candidate_support.candidate_id = (*it)->proposed_leader;
        candidate_support.total = std::count_if(it, 
                                                std::cend(votes_copy), 
                                                [&](const comms::msg::Datapad::SharedPtr &v) {
                                                    return v->proposed_leader == (*it)->proposed_leader;
                                                }
                                                );
        ballot.push_back(candidate_support);

        // Increment the iterator until the last value of the cluster is found
        for(auto last = (*it)->proposed_leader; (*++it)->proposed_leader == last;);
    }

    std::sort(ballot.begin(),
              ballot.end(),
              [](vote_count a, vote_count b){
                  return a.total < b.total;
              }
            );
    auto item = std::find_if(ballot.begin(),
                             ballot.end(),
                             [this](const vote_count &v){
                                 return v.candidate_id == this->get_ID();
                             }
                            );
    
    // If item is the last element of the ballot vector, either this candidate has won the election or there's a tie
    if((*item).total == (*ballot.cend()).total) {
        if((*item).total != (*(ballot.cend()-1)).total) { // this candidate has won
            // I've been chosen!
            this->election_timer_->cancel();
            this->election_completed_ = true;
            this->leader_elected_ = true;
            this->leader_id_ = this->id_;
            this->becomeLeader();
            this->sendHeartbeat(); // to be sure one is sent now
        } else { // tie
            // each candidate will time out and start a new election by incrementing its term and initiating another round of Request-Vote RPCs
            // Do not flag the election as completed and starts from scratch
            this->resetVotingWindow();
        }
    };
    // Otherwise, let the winning candidate send its heartbeat as leader confirmation
}

bool PelicanUnit::checkVotingTimedOut() {
    // Ensure safe access to election_timed_out
    std::lock_guard<std::mutex> lock(this->election_timedout_mutex_);
    return this->election_timed_out;
}

bool PelicanUnit::checkElectionCompleted() {
    // Ensure safe access to election_timed_out
    std::lock_guard<std::mutex> lock(this->election_completed_mutex_);
    return this->election_completed_;
}

bool PelicanUnit::checkVotingCompleted() {
    // Ensure safe access to election_timed_out
    std::lock_guard<std::mutex> lock(this->voting_completed_mutex_);
    return this->voting_completed_;
}

void PelicanUnit::setVotingTimedOut() {
    this->election_timer_->cancel();
    // Ensure safe access to election_timed_out
    std::lock_guard<std::mutex> lock(this->election_timedout_mutex_);
    this->election_timed_out = true;
}

void PelicanUnit::setVotingCompleted() {
    this->election_timer_->cancel();
    // Ensure safe access to election_timed_out
    std::lock_guard<std::mutex> lock(this->election_timedout_mutex_);
    this->election_timed_out = true;
}

void PelicanUnit::setElectionCompleted() {
    this->voting_timer->cancel();
    std::lock_guard<std::mutex> lock(this->election_completed_mutex_);
    this->election_completed_ = true;
}

void PelicanUnit::resetVotingWindow() {
    this->voting_completed_mutex_.lock();
    this->voting_completed_ = false;
    this->voting_completed_mutex_.unlock();

    this->election_timer_->reset();
    this->election_timedout_mutex_.lock();
    this->election_timed_out = false;
    this->election_timedout_mutex_.unlock();
}

void PelicanUnit::expressPreference(const comms::msg::RequestVoteRPC msg) { // TODO: think about the name
    this->sub_to_request_vote_rpc_topic_.reset();
    auto heavier = std::max_element(this->received_votes.begin(),
                                    this->received_votes.end(),
                                    [](comms::msg::Datapad::SharedPtr first, comms::msg::Datapad::SharedPtr second) {
                                        return first->candidate_mass > second->candidate_mass;
                                    }
                                    );
    this->vote((*heavier)->candidate_mass);
}

void PelicanUnit::storeCandidacy(const comms::msg::Datapad::SharedPtr msg) { // TODO: in utility file?
    std::cout << "\n\n";
    std::cout << "AGENT " << this->id_ << "RECEIVED DATAPAD DATA"   << std::endl;
    std::cout << "============================="   << std::endl;
    std::cout << "term_id: " << msg->term_id << std::endl;
    std::cout << "voter_id: " << msg->voter_id << std::endl;
    std::cout << "proposed_leader: " << msg->proposed_leader << std::endl;
    std::cout << "candidate_mass: " << msg->candidate_mass << std::endl;

    this->votes_mutex_.lock();
    this->received_votes.push_back(msg);
    this->votes_mutex_.unlock();

    // Votes received are supposedly from followers, so if no more votes come within a time frame, the elections is finished
    if(this->role_ == candidate) {
        if(this->voting_timer != nullptr) { // Make the timer restart
            this->voting_timer->reset();
        } else{
            this->voting_timer = this->create_wall_timer(this->voting_max_time_, std::bind(&PelicanUnit::setElectionCompleted, this));
        };
    };

}

void PelicanUnit::flushVotes() { // TODO: in utility file?
    // Ensure safe access to received_votes
    std::lock_guard<std::mutex> lock(this->votes_mutex_);
    this->received_votes.clear();
}
