# Development notes
From the official [Raft paper](https://raft.github.io/raft.pdf):
> The broadcast time should be an order of magnitude less than the election timeout so that leaders can
> reliably send the heartbeat messages required to keep followers from starting elections; given the randomized approach
> used for election timeouts, this inequality also makes split votes unlikely. The election timeout should be
> a few orders of magnitude less than MTBF so that the system makes steady progress

// TODO: consider mutex or interrupt for consecutive changes in role
