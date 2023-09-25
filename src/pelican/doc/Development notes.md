# Development notes
From the official [Raft paper](https://raft.github.io/raft.pdf):
> The broadcast time should be an order of magnitude less than the election timeout so that leaders can
> reliably send the heartbeat messages required to keep followers from starting elections; given the randomized approach
> used for election timeouts, this inequality also makes split votes unlikely. The election timeout should be
> a few orders of magnitude less than MTBF so that the system makes steady progress

// TODO: consider mutex or interrupt for consecutive changes in role

Following the [official guide about the migration from Gazebo classic](https://github.com/gazebosim/gz-sim/blob/gz-sim7/tutorials/migration_sdf.md#path-relative-to-an-environment-variable), I've noticed I had the wrong environmental variable setup for the models loading. I've fixed that and now, *after sourcing the local overlay,* Gazebo can successfully load each model's meshes with no problems.
As of this note creation, the correct variable used by Gazebo is `GZ_SIM_RESOURCE_PATH`.
