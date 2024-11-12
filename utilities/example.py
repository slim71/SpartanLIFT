import numpy as np
from scipy.interpolate import interp1d
import itertools
import matplotlib.pyplot as plt

# Example input data: timestamps and positions for 5 UAVs
# Assume each UAV's data is a list of [timestamp, x, y]
uav_data = {
    "uav1": np.array([[0, 1.0, 1.0], [2, 2.0, 2.0], [4, 3.0, 1.0]]),
    "uav2": np.array([[1, 2.0, 3.0], [3, 2.5, 2.5], [5, 3.5, 3.0]]),
    "uav3": np.array([[0, 0.5, 1.5], [2, 1.5, 1.8], [4, 2.5, 1.6]]),
    "uav4": np.array([[1, 1.0, 0.5], [3, 1.5, 1.0], [5, 2.0, 0.8]]),
    "uav5": np.array([[0, 1.5, 2.5], [2, 2.5, 3.0], [4, 3.5, 2.7]]),
}

# Step 1: Generate a common set of timestamps based on the range in all UAV data
all_timestamps = np.unique(np.concatenate([data[:, 0] for data in uav_data.values()]))
common_timestamps = np.linspace(all_timestamps.min(), all_timestamps.max(), num=100)

# Step 2: Interpolate the positions for each UAV at these common timestamps
interpolated_positions = {}
for uav, data in uav_data.items():
    interp_x = interp1d(data[:, 0], data[:, 1], kind="linear", fill_value="extrapolate")
    interp_y = interp1d(data[:, 0], data[:, 2], kind="linear", fill_value="extrapolate")
    interpolated_positions[uav] = np.vstack(
        (interp_x(common_timestamps), interp_y(common_timestamps))
    ).T

# Step 3: Calculate pairwise distances for each timestamp
distances = {
    pair: [] for pair in itertools.combinations(interpolated_positions.keys(), 2)
}

for t in range(len(common_timestamps)):
    for uav1, uav2 in distances.keys():
        pos1 = interpolated_positions[uav1][t]
        pos2 = interpolated_positions[uav2][t]
        distance = np.linalg.norm(pos1 - pos2)
        distances[(uav1, uav2)].append(distance)

# Step 4: Plot distances over time
plt.figure(figsize=(10, 6))
for (uav1, uav2), dist in distances.items():
    plt.plot(common_timestamps, dist, label=f"Distance {uav1}-{uav2}")
plt.xlabel("Time")
plt.ylabel("Distance")
plt.legend()
plt.title("Inter-UAV Distances Over Time")
plt.show()
