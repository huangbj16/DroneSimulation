import matplotlib.pyplot as plt
import json
import numpy as np

# filename_list = [
#     'data_pilot_p2_20240216_114911',
#     'data_pilot_p2_20240216_115354',
#     'data_pilot_p2_20240216_115612',
#     'data_pilot_p2_20240216_120118',
#     'data_pilot_p2_20240216_120452',
#     'data_pilot_p2_20240216_120640',
#     'data_pilot_p2_20240216_121706',
#     'data_pilot_p2_20240216_122224',
#     'data_pilot_p2_20240216_124427',
#     'data_pilot_p2_20240216_124536',
#     'data_pilot_p2_20240216_124818',
#     'data_pilot_p4fake_20240217_181108',
#     'data_pilot_p4fake_20240217_181545',
#     'data_pilot_p4fake_20240217_181701'
# ]

# filename_list = [
#     'data_pilot_p3_20240216_160059',
#     'data_pilot_p3_20240216_160642',
#     'data_pilot_p3_20240216_160946',
#     'data_pilot_p3_20240216_161204',
#     'data_pilot_p3_20240216_161701',
#     'data_pilot_p3_20240216_161813',
#     'data_pilot_p3_20240216_162113',
#     'data_pilot_p3_20240216_162233',
#     'data_pilot_p4fake_20240217_181108',
#     'data_pilot_p4fake_20240217_181545',
#     'data_pilot_p4fake_20240217_181701'
# ]

filename_list = [
    'data_pilot_p5_20240218_100415',
    'data_pilot_p5_20240218_101348',
    'data_pilot_p5_20240218_102927',
    'data_pilot_p5_20240218_103458',
    'data_pilot_p5_20240218_103738',
    'data_pilot_p5_20240218_104506',
    'data_pilot_p5_20240218_104819',
    'data_pilot_p5_20240218_110058',
    'data_pilot_p5_20240218_110332',
    'data_pilot_p5_20240218_111627',
    'data_pilot_p5_20240218_111925',
    'data_pilot_p5_20240218_112037',
    'data_pilot_p5_20240218_112421',
    'data_pilot_p5_20240218_112928',
    'data_pilot_p5_20240218_113044',
    'data_pilot_p5_20240218_113616',
    'data_pilot_p5_20240218_114737',
    'data_pilot_p5_20240218_114830'
]

labels = []

controller_options = ['hand', 'body']
flying_options = ['forward', 'right', 'upward']
feedback_options = ['no', 'tactile', 'assist']

import itertools

for combination in itertools.product(controller_options, flying_options, feedback_options):
    labels.append('+'.join(combination))

print(labels)


task_duration_list = []
task_distance_list = []
task_vel_mean_list = []
task_vel_std_list = []
task_input_mean_list = []
task_input_std_list = []
task_collision_count_list = []

for filename in filename_list:
    # Initialize lists to hold the data
    timestamps = []
    positions = []
    velocities = []
    collision_statuses = []
    input_diffs = []
    print('load file = ', filename)

    # Read the data from the file
    with open('results/'+filename+'.txt', 'r') as file:
        for line in file:
            data = json.loads(line)
            
            # Append data to lists
            timestamps.append(data['timestamp'])
            positions.append(data['position'])
            velocities.append(data['velocity'])
            collision_statuses.append(1 if data['collision']['has_collided'] else 0)
            if np.linalg.norm(data['input_diff']) > 100:
                print("error input", data['user_input'], data['safe_input'])
            input_diffs.append(data['input_diff'])


    # Convert timestamps to a relative scale if needed
    # For simplicity, this example uses the raw timestamps
    # Convert lists of positions and velocities to their magnitudes for easier plotting
    timestamps = np.array(timestamps)
    positions = np.array(positions)
    velocities = np.array(velocities)
    collision_statuses = np.array(collision_statuses)
    input_diffs = np.array(input_diffs)
    positions_magnitude = np.linalg.norm(positions, axis=1)
    velocities_magnitude = np.linalg.norm(velocities, axis=1)
    input_diffs_magnitude = np.linalg.norm(input_diffs, axis=1)


    ### calcualte metrics
    task_duration = (timestamps[-1] - timestamps[0])/1e9
    task_distance = np.sum(np.linalg.norm(positions[1:] - positions[:-1], axis=1))
    task_vel_mean = np.mean(velocities_magnitude)
    task_vel_std = np.std(velocities_magnitude)
    task_input_mean = np.mean(input_diffs_magnitude)
    task_input_std = np.std(input_diffs_magnitude)
    task_collision_count = np.sum(collision_statuses)

    task_duration_list.append(task_duration)
    task_distance_list.append(task_distance)
    task_vel_mean_list.append(task_vel_mean)
    task_vel_std_list.append(task_vel_std)
    task_input_mean_list.append(task_input_mean)
    task_input_std_list.append(task_input_std)
    task_collision_count_list.append(task_collision_count)
    
    # # Plotting
    # plt.clf()
    # fig, axs = plt.subplots(4, 1, figsize=(10, 15))

    # # Plot positions
    # axs[0].plot(timestamps, positions_magnitude, label='Position Magnitude')
    # axs[0].set_title('Position Over Time')
    # axs[0].set_xlabel('Timestamp')
    # axs[0].set_ylabel('Magnitude')

    # # Plot velocities
    # axs[1].plot(timestamps, velocities_magnitude, label='Velocity Magnitude', color='orange')
    # axs[1].set_title('Velocity Over Time')
    # axs[1].set_xlabel('Timestamp')
    # axs[1].set_ylabel('Magnitude')

    # # Plot collision statuses
    # axs[2].scatter(timestamps, collision_statuses, label='Collision Status', color='red')
    # axs[2].set_title('Collision Status Over Time')
    # axs[2].set_xlabel('Timestamp')
    # axs[2].set_ylabel('Collision (1=True, 0=False)')

    # # Plot input differences
    # axs[3].plot(timestamps, input_diffs_magnitude, label='Input Diff Magnitude', color='green')
    # axs[3].set_title('Input Difference Over Time')
    # axs[3].set_xlabel('Timestamp')
    # axs[3].set_ylabel('Magnitude')

    # plt.tight_layout()
    # plt.show()
    
# Now each list contains its respective variable value
print("Task Duration List:", task_duration_list)
print("Task Distance List:", task_distance_list)
print("Task Velocity Mean List:", task_vel_mean_list)
print("Task Velocity STD List:", task_vel_std_list)
print("Task Input Mean List:", task_input_mean_list)
print("Task Input STD List:", task_input_std_list)
print("Task Collision Count List:", task_collision_count_list)

titles = [
    'Task Duration',
    'Task Distance',
    'Task Velocity Mean',
    'Task Velocity STD',
    'Task Input Mean',
    'Task Input STD',
    'Task Collision Count'
]

# Data for each subplot
data = [
    task_duration_list,
    task_distance_list,
    task_vel_mean_list,
    task_vel_std_list,
    task_input_mean_list,
    task_input_std_list,
    task_collision_count_list
]

# Create subplots
# plt.clf()
fig, axs = plt.subplots(7, figsize=(10, 20))

# Plot each bar chart
for i, ax in enumerate(axs):
    ax.bar(np.arange(len(filename_list)), data[i])
    ax.set_title(titles[i])
    ax.set_ylabel('Value')
    ax.set_xticks(np.arange(len(filename_list)))
    ax.set_xticklabels(labels)
    ax.set_ylim(0, max(data[i]) + max(data[i]) * 0.1)  # Ensure the bar is visible and has some space above

plt.tight_layout()
plt.show()