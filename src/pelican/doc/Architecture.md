# Architecture

I've chosen not to create more nodes to keep a centralized architecture.
*Some request/serve function couples could be coded as ROS2 services though, so I might end up changing idea in the future.* :smiley:  

The architecture is then "single node - multi module": each node represents a whole **Agent**, which is comprised of multiple specialized modules exchanging data:
- Logger module &rarr; handles all logging actions, formatting so that the user can determine which module produced the info
- Election module &rarr; this is the core module for all actions related to the Leader Election task, based on the [Raft algorithm](https://raft.github.io/raft.pdf)
- Heartbeat module &rarr; specializes in sending, receiving and storing heartbeats
- TacMap Module &rarr; handles communication with the PX4 package

<br>
Furthermore, I've decided not to let data "flow" directly from one module to another: each of them should make a request to the main module (named **PelicanModule**) to receive data handled by another external component.

See the file [Naming convention](Naming%20convention.md) for an in-depth explanation.
