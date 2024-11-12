"""
Utility script to filter election-related logs of the fleet.
"""

import re
import os
import matplotlib as mpl
from datetime import datetime, timezone

CMAP = mpl.colormaps["Set1"]
COLORS = CMAP.colors[1:]
FILE_PATH = "/home/slim71/.ros/log/2024-10-12-18-41-58-718682-slim71-Ubuntu-81142/"
FILE_NAME = "launch.log"
N_AGENTS = 5


if __name__ == "__main__":
    leader = 0
    election_timestamp = None
    with open(os.path.join(FILE_PATH, FILE_NAME), "r", encoding="utf8") as file:
        lines = [line.rstrip() for line in file]

    # Detect leader agent
    for line in reversed(lines):
        if re.search(r"Agent \d+\|.+\|leader\|\d+", line):
            matches = re.findall(r"Agent (\d)+\|.+\|leader\|\d+", line)[0]
            leader = int(matches[0])
            break

    with open(
        os.path.join(FILE_PATH, "le_related.log"), "w", encoding="utf8"
    ) as le_file:
        to_search = [
            r"\|tbd\|",
            "heartbeat",
            r"(?:E|e)lection",
            r"(?:v|V)ote",
            r"(?:v|V)oting",
            "Becoming",
            "majority",
            r"(?:c|C)luster",
            r"(?:B|b)allot",
        ]
        for line in lines:
            # Gather positions assigned by the leader to each agent
            if re.search("pelican_", line) and any(
                re.search(string, line) for string in to_search
            ):
                le_file.write(line + "\n")

            # Detect start of Rendezvous from the leader
            if re.search("Trying to load model", line):
                agent_id = int(
                    re.findall(r"Agent (\d*).* Trying to load model", line)[0][0]
                )
                if agent_id == leader:
                    start_timestamp = datetime.fromtimestamp(
                        float(re.findall(r"\[(\d+\.\d+)\]", line)[0]), tz=timezone.utc
                    )

            # Detect end of Rendezvous
            if re.search(r"Becoming leader", line):
                agent_id = int(re.findall(r"Agent (\d*).* Becoming leader", line)[0][0])
                # If all agents have finished Rendezvous operations
                if agent_id == leader:
                    election_timestamp = datetime.fromtimestamp(
                        float(re.findall(r"\[(\d+\.\d+)\]", line)[0]), tz=timezone.utc
                    )

        # Show time elapsed before the leader is elected
        print((election_timestamp - start_timestamp).total_seconds())
