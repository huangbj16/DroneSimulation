import pandas as pd
import math

# Load the CSV file to get an overview of its structure
file_path = '../Questionnaire_results/Haptic Drone Teleoperation In-study Questionnaire.csv'
data = pd.read_csv(file_path)

# Display the first few rows of the dataframe to understand its structure
print(data.head())

data_order = []
for condition_order in data['Order of conditions']:
    condition_order_str = str(condition_order)
    condition_order = [int(condition_order_str[i])-1 for i in range(len(condition_order_str))]
    data_order.append(condition_order)

print(data_order)
# [[0, 1, 2, 3, 4], [3, 2, 0, 4, 1], [1, 3, 4, 0, 2], [2, 4, 3, 1, 0], [4, 0, 1, 2, 3], [0, 1, 2, 3, 4], [3, 2, 0, 4, 1], [1, 3, 4, 0, 2], [2, 4, 3, 1, 0], [4, 0, 1, 2, 3]]

import numpy as np

question_summaries = np.zeros((10, 5, 10))

mapping = [0, 3, 4, 1, 2]

# iterate through each row and each column by index
offset = 3
for i in range(len(data)):
    for j in range(5):
        for k in range(10):
            question_summaries[k, mapping[data_order[i][j]], i] = data.iloc[i, j*10+k+offset]

# question_means = np.mean(question_summaries, axis=2)
# question_std = np.std(question_summaries, axis=2)
# print(question_means)
# print(question_means.shape)

import matplotlib.pyplot as plt

question_texts = [
    'How mentally demanding was the task?',
    'How physically demanding was the task?',
    'How hurried or rushed was the pace of the task?',
    'How successful were you in accomplishing what you were asked to do?',
    'How hard did you have to work to accomplish your level of performance?',
    'How insecure, discouraged, irritated, stressed, and annoyed were you?',
    'How easy was it to control the drone with this input device?',
    'How much control did you feel you had over the drone?',
    'How well did the drone motion match your intention?',
    'If you felt haptic feedback, how much did the feedback help you navigate the robot?'
]

# question_texts_brief = [
#     'Mental demand',
#     'Physical demand',
#     'Temporal Demand',
#     'Performance',
#     'Effort',
#     'Frustration',
#     'Sum of Task Load',
# ]

# data_nasatlx = np.zeros((7, 5, 10))
# data_nasatlx[:6, :, :] = (question_summaries[:6, :, :] * 3 - 2) * 5
# data_nasatlx[6, :, :] = np.sum(data_nasatlx[:6, :, :], axis=0) / 6

# category_names = [
#     'NA',
#     'FSC',
#     'FSA',
#     'VSC',
#     'VSA'
# ]

# category_names_blank = [
#     '',
#     '',
#     '',
#     '',
#     ''
# ]


# import matplotlib.colors as mcolors
# # Get the RGBA value of gray
# rgba_gray = mcolors.to_rgba('gray')
# print(rgba_gray)
# # Get the colormap "Set2"
# cmap = plt.get_cmap("Paired")
# # Extract the first five colors
# colors = [cmap(i) for i in range(4)]
# colors.insert(0, rgba_gray)


# fig, axs = plt.subplots(1, 7, figsize=(20, 5))
# for i,ax in enumerate(axs):
#     ax.set_title(question_texts_brief[i])
#     ax.bar(np.arange(5), np.mean(data_nasatlx[i, :, :] ,axis=1), yerr=np.std(data_nasatlx[i, :, :], axis=1, ddof=1)/math.sqrt(10), color=colors, align='center', alpha=0.5, ecolor='black', capsize=5)
#     # ax.boxplot(data_nasatlx[i, :, :].T)
#     # ax[i%5, int(i/5)].set_title(f'Question {i+1}')
#     ax.set_xticks(np.arange(5))
#     if i == 0:
#         ax.set_xticklabels(category_names)
#         ax.set_ylabel('Score')
#     elif i == 6:
#         ax.set_xticklabels(category_names)
#         ax.set_ylabel('Average Score')
#     else:
#         ax.set_xticklabels(category_names_blank)
#     ax.set_ylim(0, 100)

# plt.tight_layout()
# plt.show()











question_texts_brief = [
    'Q1',
    'Q2',
    'Q3',
    'Q4',
    'Sense of Control Authority'
]

data_control = np.zeros((4, 5, 10))
data_control[:4, :, :] = (question_summaries[6:, :, :] * 3 - 2) * 5
# data_control[4, :, :] = np.sum(data_control[:4, :, :], axis=0) / 4

category_names = [
    'NA',
    'FSC',
    'FSA',
    'VSC',
    'VSA'
]

category_names_blank = [
    '',
    '',
    '',
    '',
    ''
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


fig, axs = plt.subplots(1, 4, figsize=(20, 5))
for i,ax in enumerate(axs):
    ax.set_title(question_texts_brief[i])
    ax.bar(np.arange(5), np.mean(data_control[i, :, :] ,axis=1), yerr=np.std(data_control[i, :, :], axis=1, ddof=1)/math.sqrt(10), color=colors, align='center', alpha=0.5, ecolor='black', capsize=5)
    # ax.boxplot(data_nasatlx[i, :, :].T)
    # ax[i%5, int(i/5)].set_title(f'Question {i+1}')
    ax.set_xticks(np.arange(5))
    if i == 0:
        ax.set_xticklabels(category_names)
        ax.set_ylabel('Score')
    elif i == 6:
        ax.set_xticklabels(category_names)
        ax.set_ylabel('Average Score')
    else:
        ax.set_xticklabels(category_names_blank)
    ax.set_ylim(0, 100)

# plt.tight_layout()
plt.show()