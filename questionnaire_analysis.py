import pandas as pd

# Load the CSV file to get an overview of its structure
file_path = '../Questionnaire_results/Haptic Drone Teleoperation In-study Questionnaire (Responses) - Form Responses 1.csv'
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

question_summaries = np.zeros((5, 10, 10))

# iterate through each row and each column by index
offset = 3
for i in range(len(data)):
    for j in range(5):
        for k in range(10):
            question_summaries[data_order[i][j], k, i] = data.iloc[i, j*10+k+offset]

# print(question_summaries)

question_means = np.mean(question_summaries, axis=2)

print(question_means)
print(question_means.shape)

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

fig, ax = plt.subplots(5, 2, figsize=(10, 20))
for i in range(10):
    ax[i%5, int(i/5)].set_title(question_texts[i])
    ax[i%5, int(i/5)].bar(np.arange(5), question_means[:, i])
    # ax[i%5, int(i/5)].set_title(f'Question {i+1}')
    ax[i%5, int(i/5)].set_xticks(np.arange(5))
    ax[i%5, int(i/5)].set_yticks(np.arange(1, 8))

plt.show()

exit()

# Define a function to map questions to their respective conditions based on the order
def map_questions_to_conditions(row, num_conditions=5):
    """
    For a given row, map each question set to its respective condition based on the order provided.
    """
    condition_order_str = str(row['Order of conditions'])
    condition_order = [int(condition_order_str[i]) for i in range(len(condition_order_str))]
    print(condition_order)
    
    # Create a dictionary to hold the sum of responses for each condition
    condition_sums = {condition: 0 for condition in range(1, num_conditions + 1)}
    total_responses_per_condition = {condition: 0 for condition in range(1, num_conditions + 1)}
    
    # Iterate over each condition and sum the responses
    for condition_idx, condition in enumerate(condition_order, start=1):
        # Identify the columns related to this condition
        condition_columns = [col for col in data.columns if col.endswith(f".{condition_idx-1}") or (col.endswith('task?') and condition_idx == 1)]
        
        # Sum the responses for this condition
        for col in condition_columns:
            condition_sums[condition] += row[col]
            total_responses_per_condition[condition] += 1
    
    # Calculate average responses for each condition
    condition_averages = {condition: (condition_sums[condition] / total_responses_per_condition[condition]) if total_responses_per_condition[condition] > 0 else 0 for condition in condition_sums}
    
    return condition_averages

# Apply the function to each row and collect the summarized data
summarized_data = data.apply(lambda row: map_questions_to_conditions(row), axis=1)

# Aggregate the summarized data across all participants for each condition
final_summary = {condition: 0 for condition in range(1, 6)}
total_participants = len(summarized_data)

for participant_data in summarized_data:
    for condition, average in participant_data.items():
        final_summary[condition] += average

# Calculate the overall average for each condition across all participants
overall_averages = {condition: final_summary[condition] / total_participants for condition in final_summary}

print(overall_averages)
