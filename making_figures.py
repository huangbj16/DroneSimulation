import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import math
import numpy as np

def read_data_csv(filename):
    data = pd.read_csv(filename)
    data = data.to_numpy().T
    # return mean and standard error
    print(data.shape)
    mean = np.mean(data, axis=1)
    sem = np.std(data, axis=1, ddof=1) / math.sqrt(data.shape[1])
    return mean, sem

# Step 1: Load your data
distance_mean, distance_std = read_data_csv('results/Task Distance.csv')
collision_duration_mean, collision_duration_std = read_data_csv('results/Task Collision Duration.csv')
input_mean, input_std = read_data_csv('results/Task Input Diff Mean.csv')


titles = [
    'Task Total Distance',
    'Task Collision Duration',
    'Task Input Disagreement'
]

# Data for each subplot
data_mean = [
    distance_mean,
    collision_duration_mean,
    input_mean
]

data_std = [
    distance_std,
    collision_duration_std,
    input_std
]

import matplotlib.colors as mcolors
# Get the RGBA value of gray
rgba_gray = mcolors.to_rgba('gray')
print(rgba_gray)
# Get the colormap "Set2"
cmap = plt.get_cmap("Paired")
# Extract the first five colors
colors = [cmap(i) for i in range(4)]
colors.insert(0, rgba_gray)
cmaps = []
for color in colors:
    cmaps.append(color)
    cmaps.append(color)
    cmaps.append(color)
print(cmaps)

# Create subplots
fig, axs = plt.subplots(len(data_mean), figsize=(10, 40))

# Custom positions
group_size = 3
space_between_groups = 1
n_groups = 5
positions = []
for i in range(n_groups):
    start_pos = i * (group_size + space_between_groups)
    positions.extend([start_pos + j for j in range(group_size)])
print(positions)

xticklabels = ['Forward', 'Right\nNA', 'Upward', '', '\nFSC', '', '', '\nFSA', '', '', '\nVSC', '', '', '\nVSA', '']
tempylabels = ['' for _ in range(15)]
ylabels = ['distance (m)', 'input difference (m/s^2)', 'collision duration (frames)']


# Plot each bar chart
for i, ax in enumerate(axs):
    bars = ax.bar(positions, data_mean[i], yerr=data_std[i], color=cmaps, align='center', alpha=0.5, ecolor='black', capsize=5)
    ax.set_title(titles[i])
    ax.set_ylabel(ylabels[i])
    ax.set_xticks(positions)
    if i == axs.size - 1:
        ax.set_xticklabels(xticklabels, ha='center')
    else:
        ax.set_xticklabels(tempylabels, ha='center')
    # ax.set_ylim(0, max(data_mean) + max(data_mean) * 0.1)  # Ensure the bar is visible and has some space above

# plt.tight_layout()
plt.show()

# for i in range(len(categories)):
#     print('category = ', categories[i])
#     for sa in task_situation_awareness_list[i]:
#         print(sa['Question1'].replace('\n', ' '))
#     print('\n')
