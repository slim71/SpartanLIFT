"""
Utility script to plot desired and real positions for the fleet during rendezvous.
"""

import re
import os
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib import gridspec
from datetime import datetime, timezone
from matplotlib.colors import Normalize
from matplotlib.animation import FuncAnimation


CMAP = mpl.colormaps["Set1"]
COLORS = CMAP.colors[1:]
FILE_PATH = "/home/slim71/.ros/log/2024-10-17-22-14-19-754759-slim71-Ubuntu-13721/"
FILE_NAME = "launch.log"
N_AGENTS = 5
XMIN, XMAX, YMIN, YMAX = -15, 1, -10, 10
TICKS_FREQUENCY = 1
ROWS = 2
COLS = N_AGENTS

x_ticks = np.arange(XMIN, XMAX + 1, TICKS_FREQUENCY)
y_ticks = np.arange(YMIN, YMAX + 1, TICKS_FREQUENCY)
x_despos = [[] for _ in range(N_AGENTS)]
y_despos = [[] for _ in range(N_AGENTS)]
x_pos = [[] for _ in range(N_AGENTS)]
y_pos = [[] for _ in range(N_AGENTS)]
timestamps = [[] for _ in range(N_AGENTS)]
npos_for_agent = {}
distances = {
    (1, 3): [],
    (1, 5): [],
    (2, 3): [],
    (2, 4): [],
    (4, 5): [],
}
dist_ts = {
    (1, 3): [],
    (1, 5): [],
    (2, 3): [],
    (2, 4): [],
    (4, 5): [],
}


if __name__ == "__main__":
    leader = 0
    start_timestamp = None
    ending_timestamp = None
    payload_pos = []

    # Read whole file
    with open(os.path.join(FILE_PATH, FILE_NAME), "r", encoding="utf8") as file:
        lines = [line.rstrip() for line in file]

    start = datetime.fromtimestamp(float(re.findall(r"^(\d+\.\d+)", lines[0])[0]))

    # Detect leader agent
    for line in reversed(lines):
        if re.search(r"Agent \d+\|.+\|leader\|\d+", line):
            matches = re.findall(r"Agent (\d)+\|.+\|leader\|\d+", line)[0]
            leader = int(matches[0])
            break

    with open(
        os.path.join(FILE_PATH, "des_rend_pos.log"), "w", encoding="utf8"
    ) as despos_file, open(
        os.path.join(FILE_PATH, "real_rend_pos.log"), "w", encoding="utf8"
    ) as realpos_file:
        for line in lines:
            # Detect payload position
            if re.search("Payload is at ", line):
                matches = re.findall(r"\((-?\d+\.\d+), (-?\d+\.\d+).+\)", line)[0]
                payload_pos.append(float(matches[0]))
                payload_pos.append(float(matches[1]))

            # Detect start of Rendezvous from the leader
            if re.search("Rendezvous operations initiated!", line):
                start_timestamp = datetime.fromtimestamp(
                    float(re.findall(r"\[(\d+\.\d+)\]", line)[0]), tz=timezone.utc
                )

            # Gather each agent's real positions
            if (
                re.search(r"Own pos at start of rendezvous iteration", line)
                and start_timestamp
                and not ending_timestamp
            ):
                timestamp, agent_id, agent_x, agent_y = re.findall(
                    r"\[(\d+\.\d+)\].+Agent (\d*).* Own pos at start of rendezvous iteration: \((-?\d+\.\d+), (-?\d+\.\d+)",
                    line,
                )[0]
                x_pos[int(agent_id) - 1].append(float(agent_x))
                y_pos[int(agent_id) - 1].append(float(agent_y))
                timestamps[int(agent_id) - 1].append(float(timestamp))
                realpos_file.write(
                    f"Agent {matches[0]}| "
                    + re.findall(r"Own pos at start of rendezvous iteration: .*", line)[
                        0
                    ]
                    + "\n"
                )

            # Gather setpoints along the way to the Rendevous point
            if (
                re.search(r"Setting position setpoint to ", line)
                and start_timestamp
                and not ending_timestamp
            ):
                agent_id, agent_x, agent_y = re.findall(
                    r"Agent (\d*).* Setting position setpoint to \((-?\d+\.\d+), (-?\d+\.\d+)",
                    line,
                )[0]
                x_despos[int(agent_id) - 1].append(float(agent_x))
                y_despos[int(agent_id) - 1].append(float(agent_y))
                despos_file.write(
                    re.findall(r"Agent (\d*).* Setting position setpoint to .*", line)[
                        0
                    ]
                    + "\n"
                )

            if re.search(r"2D distance from copter", line):
                timestamp, agent_id, from_copter, distance = re.findall(
                    r"\[(\d+\.\d+)\].+Agent (\d*).* 2D distance from copter (\d+) at \(.+\): (\d+\.\d+)",
                    line,
                )[0]
                # Rendezvous initiated
                if start_timestamp:
                    # Rendezvous not ended for agent
                    if int(agent_id) not in npos_for_agent:
                        if (int(agent_id), int(from_copter)) in distances:
                            distances[(int(agent_id), int(from_copter))].append(
                                float(distance)
                            )
                            dist_ts[(int(agent_id), int(from_copter))].append(
                                float(timestamp)
                            )

            # Detect end of Rendezvous
            if re.search(r"Finished Rendezvous successfully", line):
                agent_id = int(
                    re.findall(r"Agent (\d*).* Finished Rendezvous successfully", line)[
                        0
                    ][0]
                )
                npos_for_agent[agent_id] = len(x_pos[agent_id - 1])
                # If all agents have finished Rendezvous operations
                if len(npos_for_agent) >= N_AGENTS:
                    ending_timestamp = datetime.fromtimestamp(
                        float(re.findall(r"\[(\d+\.\d+)\]", line)[0]), tz=timezone.utc
                    )

    ######################### Normal plot ##########################
    # Create the figure and the subplots
    fig, ax = plt.subplots(1, 1)
    fig_grid = gridspec.GridSpec(ROWS, COLS, figure=fig, wspace=0, hspace=0)

    # Add axis labels
    ax.set_xlabel("$x$", size=14, labelpad=-24, x=1.02)
    ax.set_ylabel("$y$", size=14, labelpad=-21, y=1.02, rotation=0)
    # Configure colors to change for each line, using the chosen set
    ax.set_prop_cycle(color=COLORS)
    # Add ticks
    ax.set_xticks(x_ticks[x_ticks != 0])
    ax.set_yticks(y_ticks[y_ticks != 0])
    ax.set_xticks(np.arange(XMIN, XMAX + 1), minor=True)
    ax.set_yticks(np.arange(YMIN, YMAX + 1), minor=True)
    # Add a grid
    ax.grid(which="both", color="grey", linewidth=1, linestyle="-", alpha=0.2)

    # Actually plot stuff
    for id_minus_one in range(N_AGENTS):
        try:
            plt.plot(
                x_pos[id_minus_one][1:],
                y_pos[id_minus_one][1:],
                linewidth=2,
                label=f"agent{id_minus_one+1}",
            )
            plt.plot(
                x_pos[id_minus_one][0],
                y_pos[id_minus_one][0],
                "g",
                linewidth=2,
                marker="o",
            )
        except KeyError:
            continue

    plt.plot(payload_pos[0], payload_pos[1], marker="s", label="payload")

    # Set things up for the overall graphs
    plt.legend(markerscale=2, fontsize=16)
    ax.set_title("Fleet positions")

    plt.pause(0.1)  # Allows the figure to render without blocking
    plt.show(block=False)

    ######################### Gradient plot ########################
    timestamps_array = [
        np.array([(t - start.timestamp()) for t in ts]) for ts in timestamps
    ]
    min_timestamp, max_timestamp = (
        min(t[0] for t in timestamps_array if t.size),
        max(t[-1] for t in timestamps_array if t.size),
    )
    norm = Normalize(vmin=min_timestamp, vmax=max_timestamp)
    scatter_handles = []
    agent_handles = []

    # Create figure and subplots
    fig, ax = plt.subplots(1, 1)
    fig_grid = gridspec.GridSpec(ROWS, COLS, figure=fig, wspace=0, hspace=0)
    ax.set_xlabel("$x$", size=14, labelpad=-24, x=1.02)
    ax.set_ylabel("$y$", size=14, labelpad=-21, y=1.02, rotation=0)
    ax.grid(which="both", color="grey", linewidth=1, linestyle="-", alpha=0.2)

    # Plot agents with color gradient
    for id_minus_one in range(N_AGENTS):
        try:
            agent_seconds = timestamps_array[id_minus_one]
            scatter = ax.scatter(
                x_pos[id_minus_one],
                y_pos[id_minus_one],
                c=agent_seconds,
                cmap=mpl.cm.hsv,
                norm=norm,
                s=50,
            )
            scatter_handles.append(scatter)

            agent_handle = ax.plot(
                x_pos[id_minus_one][0],
                y_pos[id_minus_one][0],
                "k",
                linewidth=0,
                marker=f"${id_minus_one+1}$",
                markersize=10,
                label=f"Agent {id_minus_one + 1}",
            )[0]
            agent_handles.append(agent_handle)
        except IndexError:
            continue

    # Plot payload position
    payload_handle = ax.plot(
        payload_pos[0],
        payload_pos[1],
        "tab:brown",
        marker="s",
        label="payload",
        markersize=20,
    )[0]

    # Set up colorbar
    cbar_ax = fig.add_axes([0.92, 0.15, 0.02, 0.7])
    cbar = fig.colorbar(scatter_handles[0], cax=cbar_ax)
    cbar.set_label("Mission time (seconds)", rotation=270, labelpad=15)

    # Define ticks and labels for colorbar in seconds
    num_ticks = 5
    tick_seconds = np.linspace(min_timestamp, max_timestamp, num_ticks)
    tick_labels = [f"{int(t):,} s" for t in tick_seconds]
    cbar.set_ticks(list(map(float, tick_seconds)))
    cbar.set_ticklabels(tick_labels)

    # Create a custom legend
    handles = agent_handles + [payload_handle]
    ax.legend(
        handles=handles, loc="upper left", bbox_to_anchor=(0, 0.5), labelspacing=1.2
    )
    ax.set_title("Fleet positions with time-based color gradient")
    plt.pause(0.1)  # Allows the figure to render without blocking
    plt.show(block=False)

    ######################### Difference plot ########################
    # Create the figure and the subplots
    fig, ax = plt.subplots(1, 1)

    # Add axis labels
    ax.set_xlabel("Time [s]", size=14)  # , labelpad=-24, x=1.02)
    ax.set_ylabel("Distance [m]", size=14, labelpad=-21, y=1.02, rotation=0)
    # Configure colors to change for each line, using the chosen set
    ax.set_prop_cycle(color=mpl.colormaps["Set1"].colors)
    # Add ticks
    ax.set_xticks(np.arange(0, 40, 0.5), minor=True)
    ax.set_yticks(np.arange(0, 15, 0.5), minor=True)
    ax.set_yticks(np.arange(0, 15, 1))
    # Add a grid
    ax.grid(which="both", color="grey", linewidth=1, linestyle="-", alpha=0.2)

    for pair, distance_values in distances.items():
        # Get timestamps and convert UNIX timestamps to datetime
        timestamps = [datetime.fromtimestamp(ts) for ts in dist_ts[pair]]

        # Calculate relative times in seconds from start_time
        relative_times = [(t - start).total_seconds() for t in timestamps]

        plt.plot(relative_times, distance_values, label=f"Agents {pair}")

    plt.legend()
    plt.title("Inter-UAV Distances Over Time")
    plt.show(block=False)

    ######################### GIF ########################
    # Prepare the figure
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_xlim(XMIN, XMAX)
    ax.set_ylim(YMIN, YMAX)
    ax.set_xlabel("$x$")
    ax.set_ylabel("$y$")
    ax.grid(which="both", linestyle="--", alpha=0.5)

    # Initialize the agent and payload markers
    agents_markers = []
    for i in range(N_AGENTS):
        (agent_marker,) = ax.plot([], [], "o", markersize=10, label=f"Agent {i+1}")
        agents_markers.append(agent_marker)

    (payload_marker,) = ax.plot(
        [], [], "s", color="brown", markersize=15, label="Payload"
    )

    # Add legend
    ax.legend(loc="upper left")

    # Normalize time for color gradient
    min_timestamp, max_timestamp = (
        min(t[0] for t in timestamps_array if t.size),
        max(t[-1] for t in timestamps_array if t.size),
    )
    norm = Normalize(vmin=min_timestamp, vmax=max_timestamp)
    cmap = plt.cm.hsv

    # Update function for animation
    def update(frame):
        for i, marker in enumerate(agents_markers):
            if frame < len(x_pos[i]):
                # Update agent positions and apply color based on time
                marker.set_data(x_pos[i][frame], y_pos[i][frame])
                marker.set_color(cmap(norm(timestamps_array[i][frame])))
        if frame < len(payload_pos) // 2:
            payload_marker.set_data(payload_pos[0], payload_pos[1])
        return agents_markers + [payload_marker]

    # Total frames equal the maximum data length among agents
    total_frames = max(len(x) for x in x_pos)

    # Create animation
    ani = FuncAnimation(fig, update, frames=total_frames, interval=100, blit=True)

    # Save the animation as a GIF
    ani.save("fleet_animation.gif", writer="pillow", fps=10)

    plt.show()
