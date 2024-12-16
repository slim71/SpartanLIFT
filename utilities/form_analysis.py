"""
Utility script to plot desired and real positions for the fleet during formation control.
"""

import re
import os
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib import gridspec
from datetime import datetime, timezone
from matplotlib.colors import Normalize
import itertools
from matplotlib.animation import FuncAnimation


CMAP = mpl.colormaps["Set1"]
COLORS = CMAP.colors[1:]
FILE_PATH = "/home/slim71/.ros/log/2024-10-18-12-24-58-444498-slim71-Ubuntu-72106/"
FILE_NAME = "launch.log"
N_AGENTS = 5
TICKS_FREQUENCY = 0.5
ROWS = 2
COLS = N_AGENTS

x_despos = [[] for _ in range(N_AGENTS)]
y_despos = [[] for _ in range(N_AGENTS)]
x_pos = [[] for _ in range(N_AGENTS)]
y_pos = [[] for _ in range(N_AGENTS)]
x_setpoints = [[] for _ in range(N_AGENTS)]
y_setpoints = [[] for _ in range(N_AGENTS)]
timestamps = [[] for _ in range(N_AGENTS)]
start_timestamps = {}
plots = []
form_init_index = {}
form_ctrl_index = {}
ctrl_distances = {
    (1, 2): [],
    (1, 3): [],
    (1, 4): [],
    (3, 2): [],
    (3, 5): [],
    (4, 2): [],
    (4, 5): [],
    (5, 2): [],
}
ctrl_dist_ts = {
    (1, 2): [],
    (1, 3): [],
    (1, 4): [],
    (3, 2): [],
    (3, 5): [],
    (4, 2): [],
    (4, 5): [],
    (5, 2): [],
}
# Generate all combinations excluding cases where 2 is the first element
init_distances = {
    pair: []
    for pair in itertools.combinations(range(1, N_AGENTS + 1), 2)
    if pair[0] != 2
}
init_dist_ts = {
    pair: []
    for pair in itertools.combinations(range(1, N_AGENTS + 1), 2)
    if pair[0] != 2
}
# Add pairs where 2 is the second element
for i in range(1, N_AGENTS + 1):
    if i != 2:
        init_distances[(i, 2)] = []
        init_dist_ts[(i, 2)] = []
# Sort the dictionary by the first and second elements, then convert back to a simple dict
init_distances = dict(sorted(init_distances.items(), key=lambda x: (x[0][0], x[0][1])))
init_dist_ts = dict(sorted(init_dist_ts.items(), key=lambda x: (x[0][0], x[0][1])))


if __name__ == "__main__":
    leader = 0
    start_timestamp = None
    with open(os.path.join(FILE_PATH, FILE_NAME), "r", encoding="utf8") as file:
        lines = [line.rstrip() for line in file]

    start = datetime.fromtimestamp(
        float(re.findall(r"^(\d+\.\d+)", lines[0])[0]), tz=timezone.utc
    )

    with open(
        os.path.join(FILE_PATH, "desired_positions.log"), "w", encoding="utf8"
    ) as despos_file, open(
        os.path.join(FILE_PATH, "real_positions.log"), "w", encoding="utf8"
    ) as realpos_file, open(
        os.path.join(FILE_PATH, "setpoints.log"), "w", encoding="utf8"
    ) as setpoints_file:
        for line in reversed(lines):
            # Detect leader agent
            if re.search(r"Agent \d+\|.+\|leader\|\d+", line):
                matches = re.findall(r"Agent (\d)+\|.+\|leader\|\d+", line)
                leader = int(matches[0])
                break

        for line in lines:
            # Gather positions assigned by the leader to each agent
            if re.search(r"assigned to position:", line):
                agent_id, agent_x, agent_y = re.findall(
                    r"Agent (\d) assigned to position: \((-?\d+\.\d+), (-?\d+\.\d+)",
                    line,
                )[0]
                agent_id, agent_x, agent_y = (
                    int(agent_id),
                    float(agent_x),
                    float(agent_y),
                )

                x_despos[agent_id - 1].append(agent_x)
                y_despos[agent_id - 1].append(agent_y)
                despos_file.write(re.findall(r"Agent \d assigned .*", line)[0] + "\n")

            # Gather each agent's real positions
            if re.search(r"Sharing pos:", line):
                timestamp, agent_id, agent_x, agent_y = re.findall(
                    r"\[(\d+\.\d+)\].+Agent (\d*).* Sharing pos: \((-?\d+\.\d+), (-?\d+\.\d+)",
                    line,
                )[0]
                agent_id, agent_x, agent_y = (
                    int(agent_id),
                    float(agent_x),
                    float(agent_y),
                )
                timestamp = datetime.fromtimestamp(float(timestamp), tz=timezone.utc)

                x_pos[agent_id - 1].append(agent_x)
                y_pos[agent_id - 1].append(agent_y)
                timestamps[agent_id - 1].append(timestamp)
                realpos_file.write(
                    f"Agent {matches[0]}| "
                    + re.findall(r"Sharing pos: .*", line)[0]
                    + "\n"
                )

            # Start of formation initialization for the leader
            if re.search(r"Fleet is executing command FORMATION", line):
                start_timestamp, agent_id = re.findall(
                    r"\[(\d+\.\d+)\].+Agent (\d*).* Fleet is executing command FORMATION",
                    line,
                )[0]
                agent_id = int(agent_id)
                start_timestamp = datetime.fromtimestamp(
                    float(start_timestamp), tz=timezone.utc
                )

                if agent_id == leader:
                    form_init_index[agent_id] = len(x_pos[agent_id - 1])
                    start_timestamps[agent_id] = start_timestamp

            # Formation achieved, switching to formation control
            if re.search(r"Getting higher", line):
                agent_id = int(re.findall(r"Agent (\d*).* Getting higher", line)[0])
                form_ctrl_index[agent_id] = len(x_pos[agent_id - 1])

            # Gather each agent's setpoints
            if re.search(r"New formation position to move to:", line):
                agent_id, agent_x, agent_y = re.findall(
                    r"Agent (\d*).* New formation position to move to: \((-?\d+\.\d+), (-?\d+\.\d+)",
                    line,
                )[0]
                agent_id, agent_x, agent_y = (
                    int(agent_id),
                    float(agent_x),
                    float(agent_y),
                )

                # Register start of formation initialization for follower agents
                if agent_id not in start_timestamps:
                    form_init_index[agent_id] = len(x_pos[agent_id - 1])
                    start_timestamps[agent_id] = datetime.fromtimestamp(
                        float(re.findall(r"\[(\d+\.\d+)\]", line)[0]), tz=timezone.utc
                    )

                x_setpoints[agent_id - 1].append(agent_x)
                y_setpoints[agent_id - 1].append(agent_y)
                setpoints_file.write(
                    f"Agent {matches[0]}| "
                    + re.findall(r"New formation position to move to: .*", line)[0]
                    + "\n"
                )

            # Register inter-agent distances
            if re.search(r"2D distance from copter", line):
                timestamp, agent_id, from_copter, distance = re.findall(
                    r"\[(\d+\.\d+)\].+Agent (\d*).* 2D distance from copter (\d+) at \(.+\): (\d+\.\d+)",
                    line,
                )[0]
                agent_id, from_copter, distance = (
                    int(agent_id),
                    int(from_copter),
                    float(distance),
                )
                timestamp = datetime.fromtimestamp(float(timestamp), tz=timezone.utc)
                # print(f"agent_id: {agent_id} from_copter: {from_copter} timestamp: {timestamp}")

                # Rendezvous phase finished
                if agent_id in start_timestamps:
                    # Formation control phase already started for agent
                    if agent_id in form_ctrl_index:
                        # Register only desired pairs
                        if (agent_id, from_copter) in ctrl_distances:
                            ctrl_distances[(agent_id, from_copter)].append(distance)
                            ctrl_dist_ts[(agent_id, from_copter)].append(timestamp)
                        else:
                            continue

                    # Formation initiation phase already started for agent
                    elif (
                        agent_id in form_init_index
                        and (agent_id, from_copter) in init_distances
                    ):
                        init_distances[(agent_id, from_copter)].append(distance)
                        init_dist_ts[(agent_id, from_copter)].append(timestamp)

    timestamps_array = [
        np.array([(t - start).total_seconds() for t in ts]) for ts in timestamps
    ]

    ######################### Formation initialization positions ##########################
    min_timestamp = min(
        t[min(form_init_index.values())] for t in timestamps_array if t.size
    )
    max_timestamp = max(
        t[max(form_ctrl_index.values())] for t in timestamps_array if t.size
    )
    norm = Normalize(vmin=min_timestamp, vmax=max_timestamp)

    scatter_handles = []
    agent_handles = []
    des_handles = []

    fig, ax = plt.subplots(1, 1)
    fig_grid = gridspec.GridSpec(ROWS, COLS, figure=fig, wspace=0, hspace=0)
    ax.set_xlabel("$x$", size=14, labelpad=-24, x=1.02)
    ax.set_ylabel("$y$", size=14, labelpad=-21, y=1.02, rotation=0)
    ax.grid(which="both", color="grey", linewidth=1, linestyle="-", alpha=0.2)

    # Plot agents with color gradient
    for id_minus_one in range(N_AGENTS):
        try:
            agent_seconds = timestamps_array[id_minus_one]
            start_idx = form_init_index[id_minus_one + 1] + 1
            end_idx = form_ctrl_index[id_minus_one + 1]

            scatter = ax.scatter(
                x_pos[id_minus_one][start_idx:end_idx],
                y_pos[id_minus_one][start_idx:end_idx],
                c=agent_seconds[start_idx:end_idx],
                cmap=mpl.cm.hsv,
                norm=norm,
                s=50,
            )
            scatter_handles.append(scatter)

            # Highlight starting position
            label = (
                f"Agent {id_minus_one + 1} start pos (L)"
                if leader == id_minus_one + 1
                else f"Agent {id_minus_one + 1} start pos"
            )
            agent_handle = ax.plot(
                x_pos[id_minus_one][form_init_index[id_minus_one + 1]],
                y_pos[id_minus_one][form_init_index[id_minus_one + 1]],
                "forestgreen",
                linewidth=0,
                marker="s",
                markersize=12,
                label=None,
            )[0]
            agent_handle = ax.plot(
                x_pos[id_minus_one][form_init_index[id_minus_one + 1]],
                y_pos[id_minus_one][form_init_index[id_minus_one + 1]],
                "lime",
                linewidth=0,
                marker=f"${id_minus_one+1}$",
                markersize=10,
                label=label,
            )[0]
            agent_handles.append(agent_handle)

            # Highlight desired position
            des_handle = ax.plot(
                x_despos[id_minus_one][1],
                y_despos[id_minus_one][1],
                "k",
                linewidth=0,
                marker=f"${id_minus_one+1}$",
                markersize=10,
                label=f"Agent {id_minus_one+1} des pos",
            )[0]
            des_handles.append(des_handle)

        except KeyError:
            continue  # Skip any agents missing from form_init_index or form_ctrl_index

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
    handles = agent_handles + des_handles
    ax.legend(
        handles=handles, loc="lower left", bbox_to_anchor=(-0.16, 0), labelspacing=1.2
    )
    ax.set_title("Formation initialization positions (real)")
    plt.pause(0.1)
    # plt.show()

    ######################## Formation initialization gif ##########################
    # Create a new figure for the GIF animation
    fig_anim, ax_anim = plt.subplots()
    ax_anim.set_xlabel("$x$", size=14)
    ax_anim.set_ylabel("$y$", size=14)
    ax_anim.grid(which="both", color="grey", linewidth=1, linestyle="-", alpha=0.2)

    # Set axis limits (optional, to match your existing plot)
    x_min = -8.5
    x_max = -3
    y_min = -2.5
    y_max = 3
    ax_anim.set_xlim(x_min, x_max)
    ax_anim.set_ylim(y_min, y_max)

    # Initialize scatter plots for each agent with correct color range
    scatter_anim = []
    for id_minus_one in range(N_AGENTS):
        try:
            # Define the relevant range
            start_idx = form_init_index[id_minus_one + 1] + 1
            end_idx = form_ctrl_index[id_minus_one + 1]
            agent_seconds = timestamps_array[id_minus_one][start_idx:end_idx]

            # Initialize scatter with the correct color mapping
            scatter = ax_anim.scatter([], [], c=[], cmap=mpl.cm.hsv, norm=norm, s=50)
            scatter_anim.append(scatter)
        except KeyError:
            scatter_anim.append(None)  # If no data, append None for this agent

    # Determine the maximum number of frames to animate based on [start_idx:end_idx]
    relevant_lengths = [
        form_ctrl_index[i + 1] - (form_init_index[i + 1] + 1) for i in range(N_AGENTS)
    ]
    max_frames = max(relevant_lengths)

    text_labels = []
    for id_minus_one in range(N_AGENTS):
        text = ax_anim.text(
            0,
            0,
            str(id_minus_one + 1),  # Agent ID as the text
            color="black",
            ha="center",
            va="center",
            fontsize=12,
            fontweight="bold",
        )
        text_labels.append(text)

    # Update function for animation: display a colored number instead of a point
    def update(frame):
        for id_minus_one, text in enumerate(text_labels):
            try:
                # Use the relevant data range [start_idx:end_idx]
                start_idx = form_init_index[id_minus_one + 1] + 1
                end_idx = form_ctrl_index[id_minus_one + 1]

                if frame < end_idx - start_idx:
                    # Get the current position and color for the agent
                    x_data = x_pos[id_minus_one][start_idx + frame]
                    y_data = y_pos[id_minus_one][start_idx + frame]
                    c_data = timestamps_array[id_minus_one][start_idx + frame]

                    # Update the text position and color
                    text.set_position((x_data, y_data))
                    text.set_color(mpl.cm.hsv(norm(c_data)))  # Use color map with norm
            except IndexError:
                continue
        return text_labels

    # Set up animation
    ani = FuncAnimation(fig_anim, update, frames=max_frames, interval=100, blit=True)

    # Save the animation as a GIF
    ani.save("fleet_evolution.gif", writer="pillow", fps=10)

    print("GIF saved as 'fleet_evolution.gif'")

    ######################## Formation initialization distances ##########################
    min_timestamp = min(t[0] for t in init_dist_ts.values() if t)
    max_timestamp = max(t[-1] for t in init_dist_ts.values() if t)

    # Create the figure and the subplots
    fig, axes = plt.subplots(len(init_distances), 1, sharex=True)
    # Set a color cycle
    colors = mpl.colormaps["tab10"].colors

    for i, (pair, distance_values) in enumerate(init_distances.items()):
        # Get timestamps and convert UNIX timestamps to datetime
        stamps = init_dist_ts[pair]

        # Calculate relative times in seconds from start_time
        relative_times = [(t - start).total_seconds() for t in stamps]

        # Plot on the corresponding subplot
        ax = axes[i]
        ax.plot(
            relative_times,
            distance_values,
            color=colors[i % len(colors)],
            label=f"Agents {pair}",
        )

        # Set grid
        ax.set_yticks(np.arange(0, 5, 1))
        ax.grid(which="both", color="grey", linewidth=1, linestyle="-", alpha=0.2)
        ax.legend(loc="lower right")

    # Shared x-axis label
    axes[-1].set_xlabel("Time [s]", size=14)
    # Shared y-axis label
    fig.text(0.1, 0.5, "Distance [m]", va="center", rotation="vertical", size=14)

    plt.suptitle("Inter-UAV Distances Over Time During Formation Initialization")
    plt.pause(0.1)
    plt.show(block=False)

    ######################### Formation control positions ##########################
    timestamps_array = [
        np.array([(t - start).total_seconds() for t in ts]) for ts in timestamps
    ]
    min_timestamp = min(
        t[min(form_ctrl_index.values())] for t in timestamps_array if t.size
    )
    max_timestamp = max(t[-1] for t in timestamps_array if t.size)
    norm = Normalize(vmin=min_timestamp, vmax=max_timestamp)
    norm = Normalize(vmin=min_timestamp, vmax=max_timestamp)

    scatter_handles = []
    agent_handles = []
    des_handles = []

    fig, ax = plt.subplots(1, 1)
    fig_grid = gridspec.GridSpec(ROWS, COLS, figure=fig, wspace=0, hspace=0)
    ax.set_xlabel("$x$", size=14, labelpad=-24, x=1.02)
    ax.set_ylabel("$y$", size=14, labelpad=-21, y=1.02, rotation=0)
    ax.grid(which="both", color="grey", linewidth=1, linestyle="-", alpha=0.2)

    for id_minus_one in range(N_AGENTS):
        try:
            agent_seconds = timestamps_array[id_minus_one]
            scatter = ax.scatter(
                x_pos[id_minus_one][form_ctrl_index[id_minus_one + 1] + 1 :],
                y_pos[id_minus_one][form_ctrl_index[id_minus_one + 1] + 1 :],
                c=agent_seconds[form_ctrl_index[id_minus_one + 1] + 1 :],
                cmap=mpl.cm.hsv,
                norm=norm,
                s=50,
            )
            scatter_handles.append(scatter)

            # Highlight starting position
            label = f"Agent {id_minus_one + 1} start pos" + (
                " (L)" if leader == id_minus_one + 1 else ""
            )
            agent_handle = ax.plot(
                x_pos[id_minus_one][form_ctrl_index[id_minus_one + 1]],
                y_pos[id_minus_one][form_ctrl_index[id_minus_one + 1]],
                "navy",
                linewidth=0,
                marker="s",
                markersize=15,
                label=None,
            )[0]
            agent_handle = ax.plot(
                x_pos[id_minus_one][form_ctrl_index[id_minus_one + 1]],
                y_pos[id_minus_one][form_ctrl_index[id_minus_one + 1]],
                "aqua",
                linewidth=0,
                marker=f"${id_minus_one+1}$",
                markersize=10,
                label=label,
            )[0]
            agent_handles.append(agent_handle)

            # Highlight desired position
            des_handle = ax.plot(
                x_despos[id_minus_one][2:],
                y_despos[id_minus_one][2:],
                "k-",
                linewidth=2,
                markersize=10,
                marker=f"${id_minus_one+1}$",
                label=f"Agent {id_minus_one+1} des pos",
                markevery=[0],
            )[0]
            des_handles.append(des_handle)

        except KeyError:
            continue

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
    handles = agent_handles + des_handles
    ax.legend(
        handles=handles, loc="lower left", bbox_to_anchor=(-0.16, 0), labelspacing=1.2
    )

    ax.set_title("Formation control positions (real)")
    plt.legend()
    plt.pause(0.1)
    plt.show(block=False)

    ######################### Formation control gif ##########################
    # Create a new figure for the GIF animation
    fig_anim, ax_anim = plt.subplots()
    ax_anim.set_xlabel("$x$", size=14)
    ax_anim.set_ylabel("$y$", size=14)
    ax_anim.grid(which="both", color="grey", linewidth=1, linestyle="-", alpha=0.2)

    # Set axis limits (optional, to match your existing plot)
    x_min = -8.5
    x_max = -1
    y_min = -2.5
    y_max = 6
    ax_anim.set_xlim(x_min, x_max)
    ax_anim.set_ylim(y_min, y_max)

    # Initialize scatter plots for each agent with correct color range
    scatter_anim = []
    for id_minus_one in range(N_AGENTS):
        try:
            # Define the relevant range
            start_idx = form_ctrl_index[id_minus_one + 1] + 1
            end_idx = len(x_pos[id_minus_one])
            agent_seconds = timestamps_array[id_minus_one][start_idx:end_idx]

            # Initialize scatter with the correct color mapping
            scatter = ax_anim.scatter([], [], c=[], cmap=mpl.cm.hsv, norm=norm, s=50)
            scatter_anim.append(scatter)
        except KeyError:
            scatter_anim.append(None)  # If no data, append None for this agent

    # Determine the maximum number of frames to animate based on [start_idx:end_idx]
    relevant_lengths = [
        form_ctrl_index[i + 1] - (form_init_index[i + 1] + 1) for i in range(N_AGENTS)
    ]
    max_frames = max(relevant_lengths)

    text_labels = []
    for id_minus_one in range(N_AGENTS):
        text = ax_anim.text(
            0,
            0,
            str(id_minus_one + 1),  # Agent ID as the text
            color="black",
            ha="center",
            va="center",
            fontsize=12,
            fontweight="bold",
        )
        text_labels.append(text)

    # Update function for animation: display a colored number instead of a point
    def update(frame):
        for id_minus_one, text in enumerate(text_labels):
            try:
                # Use the relevant data range [start_idx:end_idx]
                start_idx = form_ctrl_index[id_minus_one + 1] + 1
                end_idx = len(x_pos[id_minus_one])

                if frame < end_idx - start_idx:
                    # Get the current position and color for the agent
                    x_data = x_pos[id_minus_one][start_idx + frame]
                    y_data = y_pos[id_minus_one][start_idx + frame]
                    c_data = timestamps_array[id_minus_one][start_idx + frame]

                    # Update the text position and color
                    text.set_position((x_data, y_data))
                    text.set_color(mpl.cm.hsv(norm(c_data)))  # Use color map with norm
            except IndexError:
                continue
        return text_labels

    # Set up animation
    ani = FuncAnimation(fig_anim, update, frames=max_frames, interval=100, blit=True)

    # Save the animation as a GIF
    ani.save("fleet_evolution.gif", writer="pillow", fps=10)

    print("GIF saved as 'fleet_evolution.gif'")

    ######################### Formation control distances ##########################
    min_timestamp = min(t[0] for t in ctrl_dist_ts.values() if t)
    max_timestamp = max(t[-1] for t in ctrl_dist_ts.values() if t)

    # Create the figure and the subplots
    fig, axes = plt.subplots(len(ctrl_distances), 1, sharex=True)
    # Set a color cycle
    colors = mpl.colormaps["tab10"].colors

    for i, (pair, distance_values) in enumerate(ctrl_distances.items()):
        # Get timestamps and convert UNIX timestamps to datetime
        stamps = ctrl_dist_ts[pair]

        # Calculate relative times in seconds from start_time
        relative_times = [(t - start).total_seconds() for t in stamps]

        # Plot on the corresponding subplot
        ax = axes[i]
        ax.plot(
            relative_times,
            distance_values,
            color=colors[i % len(colors)],
            label=f"Agents {pair}",
        )

        # Set grid
        ax.set_yticks(np.arange(0, 5, 1))
        ax.grid(which="both", color="grey", linewidth=1, linestyle="-", alpha=0.2)
        ax.legend(loc="lower right")

    # Shared x-axis label
    axes[-1].set_xlabel("Time [s]", size=14)
    # Shared y-axis label
    fig.text(0.1, 0.5, "Distance [m]", va="center", rotation="vertical", size=14)

    plt.suptitle("Inter-UAV Distances Over Time During Formation Control")
    plt.show(block=False)
