import matplotlib.pyplot as plt
import json

# Initialize lists to hold the data
timestamps = []
positions = []
velocities = []
collision_statuses = []
input_diffs = []

# Read the data from the file
with open('results/data_20240211_124825.txt', 'r') as file:
    for line in file:
        data = json.loads(line)
        
        # Append data to lists
        timestamps.append(data['timestamp'])
        positions.append(data['position'])
        velocities.append(data['velocity'])
        collision_statuses.append(1 if data['collision']['has_collided'] else 0)
        input_diffs.append(data['input_diff'])

# Convert timestamps to a relative scale if needed
# For simplicity, this example uses the raw timestamps
# Convert lists of positions and velocities to their magnitudes for easier plotting
positions_magnitude = [((x**2 + y**2 + z**2)**0.5) for x, y, z in positions]
velocities_magnitude = [((x**2 + y**2 + z**2)**0.5) for x, y, z in velocities]
input_diffs_magnitude = [((x**2 + y**2 + z**2)**0.5) for x, y, z in input_diffs]

# Plotting
fig, axs = plt.subplots(4, 1, figsize=(10, 15))

# Plot positions
axs[0].plot(timestamps, positions_magnitude, label='Position Magnitude')
axs[0].set_title('Position Over Time')
axs[0].set_xlabel('Timestamp')
axs[0].set_ylabel('Magnitude')

# Plot velocities
axs[1].plot(timestamps, velocities_magnitude, label='Velocity Magnitude', color='orange')
axs[1].set_title('Velocity Over Time')
axs[1].set_xlabel('Timestamp')
axs[1].set_ylabel('Magnitude')

# Plot collision statuses
axs[2].scatter(timestamps, collision_statuses, label='Collision Status', color='red')
axs[2].set_title('Collision Status Over Time')
axs[2].set_xlabel('Timestamp')
axs[2].set_ylabel('Collision (1=True, 0=False)')

# Plot input differences
axs[3].plot(timestamps, input_diffs_magnitude, label='Input Diff Magnitude', color='green')
axs[3].set_title('Input Difference Over Time')
axs[3].set_xlabel('Timestamp')
axs[3].set_ylabel('Magnitude')

plt.tight_layout()
plt.show()
