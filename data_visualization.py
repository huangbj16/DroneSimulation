import matplotlib.pyplot as plt
import json
import numpy as np
import os
import re

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

categories = [
    'body_forward_False_False',
    'body_right_False_False',
    'body_upward_False_False',
    'body_forward_True_False',
    'body_right_True_False',
    'body_upward_True_False',
    'body_forward_True_True',
    'body_right_True_True',
    'body_upward_True_True',
    'hand_forward_True_False',
    'hand_right_True_False',
    'hand_upward_True_False',
    'hand_forward_True_True',
    'hand_right_True_True',
    'hand_upward_True_True'
]

filenames_folder = os.listdir('results')
filename_list = []
for filename in filenames_folder:
    if 'formal' in filename and 'formalp9' in filename:
        # print(filename)
        filename_list.append(filename)

# labels = []

# controller_options = ['hand', 'body']
# flying_options = ['forward', 'right', 'upward']
# feedback_options = ['False_False', 'True_False', 'True_True']

# import itertools

# for combination in itertools.product(controller_options, flying_options, feedback_options):
#     labels.append('_'.join(combination))


task_duration_list = [[] for _ in range(len(categories))]
task_distance_list = [[] for _ in range(len(categories))]
task_vel_mean_list = [[] for _ in range(len(categories))]
task_vel_std_list = [[] for _ in range(len(categories))]
task_input_mean_list = [[] for _ in range(len(categories))]
task_input_std_list = [[] for _ in range(len(categories))]
task_collision_duration_list = [[] for _ in range(len(categories))]
task_collision_count_list = [[] for _ in range(len(categories))]

# task_duration_list = []
# task_distance_list = []
# task_vel_mean_list = []
# task_vel_std_list = []
# task_input_mean_list = []
# task_input_std_list = []
# task_collision_count_list = []

'''
    funciton of collision_filter
    remove repeat count of collision
    1. "has_collided": true -> means it is a collision
    2. if the index of the collision in the list is 0, then do not count
    3. when there is a collision, then count += 1; set it as the current collision time and current object_id
    4. for any following collision happened in the next 1.0s with the same object_id, then do not count
    5. if the next collision is 1.0s after the last collision, then count += 1; set it as the current collision time and current object_id
    6. if the next collision is with a different object, then count += 1; set it as the current collision time and current object_id
'''
def collision_filter(collision_list):
    count = 0
    last_collision_time = 0
    last_object_id = 0
    for i in range(len(collision_list)):
        if collision_list[i]['has_collided'] and i > 0:
            if last_collision_time == 0:
                last_collision_time = collision_list[i]['time_stamp']
                last_object_id = collision_list[i]['object_id']
                count += 1
            elif collision_list[i]['time_stamp'] - last_collision_time > 1.0e9:
                last_collision_time = collision_list[i]['time_stamp']
                last_object_id = collision_list[i]['object_id']
                count += 1
            elif collision_list[i]['object_id'] != last_object_id:
                last_collision_time = collision_list[i]['time_stamp']
                last_object_id = collision_list[i]['object_id']
                count += 1
            else:
                last_collision_time = collision_list[i]['time_stamp']

    return count


for filename in filename_list:
    # Initialize lists to hold the data
    timestamps = []
    positions = []
    velocities = []
    collision_statuses = []
    controller_inputs = []
    input_diffs = []
    collision_list = []
    print('load file = ', filename)

    # find category of the filename
    pattern = r"data_\d{8}_\d{6}_formalp\d+_(.*).txt"
    category_name = re.sub(pattern, r'\1', filename)
    # print('category_name = ', category_name)
    category_index = categories.index(category_name)
    # print('category_index = ', category_index)

    # Read the data from the file
    with open('results/'+filename, 'r') as file:
        for line in file:
            data = json.loads(line)
            if 'control_mode' in data.keys():
                control_mode = data['control_mode']
                fly_mode = data['fly_mode']
                if fly_mode == 'forward':
                    direction_vector = np.array([1, 0, 0])
                elif fly_mode == 'right':
                    direction_vector = np.array([0, 1, 0])
                elif fly_mode == 'upward':
                    direction_vector = np.array([0, 0, 1])
                else:
                    print('error fly_mode')
                    exit(-1)
                continue # skip the first line
            if 'situation_awareness' in data.keys():
                print('situation_awareness', data['situation_awareness'])
                continue
            # Append data to lists
            timestamps.append(data['timestamp'])
            positions.append(data['position'])
            velocities.append(data['velocity'])
            collision_statuses.append(1 if data['collision']['has_collided'] else 0)
            collision_list.append(data['collision'])
            controller_inputs.append(data['controller_input'])
            if np.linalg.norm(data['input_diff']) > 100:
                print("error input", data['optimization'], data['user_input'], data['safe_input'])
                input_diffs.append([0, 0, 0])
            else:
                input_diffs.append(data['input_diff'])

    # crop data from start command to finish
    start_frame = -1
    end_frame = len(timestamps)+1
    for i in range(len(timestamps)):
        if np.linalg.norm(controller_inputs[i]) > 1e-3 and start_frame == -1:
            start_frame = i
        if np.dot(positions[i], direction_vector) > 50.0 and end_frame == len(timestamps)+1:
            end_frame = i
    assert start_frame != -1
    assert end_frame != len(timestamps)+1
    print('start_frame = ', start_frame, 'end_frame = ', end_frame)
    timestamps = timestamps[start_frame:end_frame]
    positions = positions[start_frame:end_frame]
    velocities = velocities[start_frame:end_frame]
    collision_statuses = collision_statuses[start_frame:end_frame]
    input_diffs = input_diffs[start_frame:end_frame]
    controller_inputs = controller_inputs[start_frame:end_frame]
    collision_list = collision_list[start_frame:end_frame]
    collision_count = collision_filter(collision_list)

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
    task_collision_duration = np.sum(collision_statuses)

    task_duration_list[category_index].append(task_duration)
    task_distance_list[category_index].append(task_distance)
    task_vel_mean_list[category_index].append(task_vel_mean)
    task_vel_std_list[category_index].append(task_vel_std)
    task_input_mean_list[category_index].append(task_input_mean)
    task_input_std_list[category_index].append(task_input_std)
    task_collision_duration_list[category_index].append(task_collision_duration)
    task_collision_count_list[category_index].append(collision_count)
    
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
# print("Task Duration List:", task_duration_list)
# print("Task Distance List:", task_distance_list)
# print("Task Velocity Mean List:", task_vel_mean_list)
# print("Task Velocity STD List:", task_vel_std_list)
# print("Task Input Mean List:", task_input_mean_list)
# print("Task Input STD List:", task_input_std_list)
# print("Task Collision Count List:", task_collision_count_list)

titles = [
    'Task Duration',
    'Task Distance',
    'Task Velocity Mean',
    'Task Velocity STD',
    'Task Input Diff Mean',
    'Task Input Diff STD',
    'Task Collision Duration',
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
    task_collision_duration_list,
    task_collision_count_list
]


print(task_collision_count_list)

def generate_gradient_colormaps(base_colors):
    """
    Generates a list of gradient colormaps for each base color, transitioning from light to dark.
    """
    gradient_colormaps = {}
    for color_name, base_color in base_colors.items():
        # Define the color gradient: start with a lighter version of the base color and end with a darker version
        colors = [base_color + np.array([0.2, 0.2, 0.2, 0]),  # Lighter
                  base_color + np.array([0.1, 0.1, 0.1, 0]),  # Slightly lighter
                  base_color,  # Base color
                  base_color - np.array([0.1, 0.1, 0.1, 0]),  # Slightly darker
                  base_color - np.array([0.2, 0.2, 0.2, 0])]  # Darker
        # Ensure color values are within valid range [0, 1]
        colors = np.clip(colors, 0, 1)
        gradient_colormaps[color_name] = colors
    return gradient_colormaps

# Base colors for Blue, Orange, Green from 'tab10' colormap
base_colors = {
    'Blue': np.array([0.12156863, 0.46666667, 0.70588235, 1]),
    'Orange': np.array([1.0, 0.49803922, 0.05490196, 1]),
    'Green': np.array([0.17254902, 0.62745098, 0.17254902, 1])
}

# Generate gradient colormaps for each color
gradient_colormaps = generate_gradient_colormaps(base_colors)

cmaps = []

for i, (color_name, gradients) in enumerate(gradient_colormaps.items()):
    for j, color in enumerate(gradients):
        cmaps.append(color)

cmaps = np.array(cmaps).reshape(3, 5, 4)
cmaps = cmaps.transpose(1, 0, 2).reshape(15, 4)

# Create subplots
# plt.clf()
fig, axs = plt.subplots(8, figsize=(10, 40))

# Plot each bar chart
for i, ax in enumerate(axs):
    data_mean = [np.mean(data[i][j]) for j in range(len(categories))]
    data_std = [np.std(data[i][j]) for j in range(len(categories))]
    ax.bar(np.arange(len(categories)), data_mean, yerr=data_std, color=cmaps, align='center', alpha=0.5, ecolor='black', capsize=10)
    ax.set_title(titles[i])
    ax.set_ylabel('Value')
    ax.set_xticks(np.arange(len(categories)))
    if i == axs.size - 1:
        ax.set_xticklabels(categories, rotation=45, ha='right')
    # ax.set_ylim(0, max(data_mean) + max(data_mean) * 0.1)  # Ensure the bar is visible and has some space above

plt.tight_layout()
plt.show()