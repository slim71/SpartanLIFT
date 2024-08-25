"""
Utility script to plot desired and real positions for the fleet.
"""

import re
import os
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib import gridspec


CMAP = mpl.colormaps["Set1"]
COLORS = CMAP.colors[1:]
FILE_PATH = "/home/slim71/.ros/log/2024-08-24-18-31-11-075701-slim71-Ubuntu-35197/"
FILE_NAME = "launch.log"
N_AGENTS = 5
XMIN, XMAX, YMIN, YMAX = -9, -3, -3, 3
TICKS_FREQUENCY = 0.5
ROWS = 2
COLS = N_AGENTS

x_ticks = np.arange(XMIN, XMAX + 1, TICKS_FREQUENCY)
y_ticks = np.arange(YMIN, YMAX + 1, TICKS_FREQUENCY)
x_despos = [[] for _ in range(N_AGENTS)]
y_despos = [[] for _ in range(N_AGENTS)]
x_pos = [[] for _ in range(N_AGENTS)]
y_pos = [[] for _ in range(N_AGENTS)]
x_setpoints = [[] for _ in range(N_AGENTS)]
y_setpoints = [[] for _ in range(N_AGENTS)]
plots = []
form_index = {}


if __name__ == "__main__":
    leader = 0
    with open(os.path.join(FILE_PATH, FILE_NAME), "r", encoding="utf8") as file:
        lines = [line.rstrip() for line in file]

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
                matches = re.findall(
                    r"Agent (\d) assigned to position: \((-*\d+\.\d+), (-*\d+\.\d+)",
                    line,
                )[0]
                x_despos[int(matches[0]) - 1].append(float(matches[1]))
                y_despos[int(matches[0]) - 1].append(float(matches[2]))
                despos_file.write(re.findall(r"Agent \d assigned .*", line)[0] + "\n")

            # Gather each agent's real positions
            if re.search(r"Sharing pos:", line):
                matches = re.findall(
                    r"Agent (\d*).* Sharing pos: \((-*\d+\.\d+), (-*\d+\.\d+)",
                    line,
                )[0]
                x_pos[int(matches[0]) - 1].append(float(matches[1]))
                y_pos[int(matches[0]) - 1].append(float(matches[2]))
                realpos_file.write(
                    f"Agent {matches[0]}| "
                    + re.findall(r"Sharing pos: .*", line)[0]
                    + "\n"
                )
            if re.search(r"Finished Rendezvous successfully", line):
                matches = re.findall(
                    r"Agent (\d*).* Finished Rendezvous successfully", line
                )[0]
                form_index[int(matches[0])] = len(x_pos[int(matches[0]) - 1])

            # Gather each agent's setpoints
            if re.search(r"New formation position to move to:", line):
                matches = re.findall(
                    r"Agent (\d*).* New formation position to move to: \((-*\d+\.\d+), (-*\d+\.\d+)",
                    line,
                )[0]
                x_setpoints[int(matches[0]) - 1].append(float(matches[1]))
                y_setpoints[int(matches[0]) - 1].append(float(matches[2]))
                setpoints_file.write(
                    f"Agent {matches[0]}| "
                    + re.findall(r"New formation position to move to: .*", line)[0]
                    + "\n"
                )

    ######################### Plots setup ##########################

    # Create the figure and the subplots
    fig = plt.figure()
    fig_grid = gridspec.GridSpec(ROWS, COLS, figure=fig, wspace=0, hspace=0)
    plots.append(fig.add_subplot(fig_grid[0, :2]))
    plots.append(fig.add_subplot(fig_grid[0, 3:]))
    for i in range(N_AGENTS):
        plots.append(fig.add_subplot(fig_grid[1, i]))

    # Configure each subplot
    for plot in plots:
        # Focus on the plot at hand
        plt.sca(plot)
        # Set the portion of axis to show
        plot.set(xlim=(XMIN - 1, XMAX + 1), ylim=(YMIN - 1, YMAX + 1), aspect="equal")
        # Spines are the surrounding lines of a plot, the "bounding box"
        plot.spines["bottom"].set_position("zero")
        plot.spines["left"].set_position("zero")
        plot.spines["top"].set_visible(False)
        plot.spines["right"].set_visible(False)
        # Add axis labels
        plot.set_xlabel("$x$", size=14, labelpad=-24, x=1.02)
        plot.set_ylabel("$y$", size=14, labelpad=-21, y=1.02, rotation=0)
        # Configure colors to change for each line, using the chosen set
        plot.set_prop_cycle(color=COLORS)
        # Add ticks
        plot.set_xticks(x_ticks[x_ticks != 0])
        plot.set_yticks(y_ticks[y_ticks != 0])
        plot.set_xticks(np.arange(XMIN, XMAX + 1), minor=True)
        plot.set_yticks(np.arange(YMIN, YMAX + 1), minor=True)
        # Add a grid
        plot.grid(which="both", color="grey", linewidth=1, linestyle="-", alpha=0.2)

    # Actually plot stuff
    for i in range(N_AGENTS):
        ###### Plot everything together
        # Desired positions
        if len(x_despos[i]) > 0:
            plots[0].plot(
                x_despos[i][1:], y_despos[i][1:], linewidth=2, label=f"agent{i+1}"
            )
            plots[0].plot(x_despos[i][0], y_despos[i][0], "b", linewidth=2, marker="o")

        # Real positions
        try:
            plots[1].plot(
                x_pos[i][form_index[i + 1] + 1 :],
                y_pos[i][form_index[i + 1] + 1 :],
                linewidth=2,
                label=f"agent{i+1}",
            )
            plots[1].plot(
                x_pos[i][form_index[i + 1]],
                y_pos[i][form_index[i + 1]],
                "g",
                linewidth=2,
                marker="o",
            )
        except KeyError:
            continue

        ###### Separate each agent's plot
        # Desired positions
        if len(x_despos[i]) > 0:
            plots[i + 2].plot(
                x_despos[i][1:], y_despos[i][1:], "b", linewidth=2, label="desired"
            )
            plots[i + 2].plot(
                x_despos[i][0], y_despos[i][0], "b", linewidth=2, marker="o"
            )

        # Real positions
        try:
            plots[i + 2].plot(
                x_pos[i][form_index[i + 1] + 1 :],
                y_pos[i][form_index[i + 1] + 1 :],
                "g",
                linewidth=2,
                label="real",
            )
            plots[i + 2].plot(
                x_pos[i][form_index[i + 1]],
                y_pos[i][form_index[i + 1]],
                "g",
                linewidth=2,
                marker="o",
            )
            plots[i + 2].plot(
                x_setpoints[i],
                y_setpoints[i],
                "c",
                linewidth=2,
                marker="2",
                label="setpoints",
            )
            plots[i + 2].plot(
                x_setpoints[i][0],
                y_setpoints[i][0],
                "c",
                linewidth=2,
                marker="o",
            )
        except (KeyError, IndexError):
            pass

        # Configuration
        plots[i + 2].set_title(f"Agent {i+1}{' (leader)' if leader == i+1 else ''}")
        plots[i + 2].legend()

    # Set things up for the overall graphs
    plots[0].legend()
    plots[0].set_title("All desired positions")
    plots[1].legend()
    plots[1].set_title("All real position free")

    # Configure the layout and show the figure
    plt.tight_layout(pad=0.4, w_pad=0.5, h_pad=0.5)
    plt.subplots_adjust(right=0.98, left=0.02, top=0.95, bottom=0.02)
    plt.show()
