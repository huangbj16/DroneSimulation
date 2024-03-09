import pygame
import pygame_gui
import json
import numpy as np

import subprocess
import time

cubes_path = "D:/2023Fall/DroneSimulation/TestSceneBright/WindowsNoEditor/Blocks/Content/Settings"
cubes_filename = "cubes.txt"
flymode_filenames = {
    "forward": ["cubes_formalstudy_forward_1.txt", "cubes_formalstudy_forward_2.txt", "cubes_formalstudy_forward_3.txt"],
    "right": ["cubes_formalstudy_right_1.txt", "cubes_formalstudy_right_2.txt", "cubes_formalstudy_right_3.txt"],
    "upward": ["cubes_formalstudy_upward_1.txt", "cubes_formalstudy_upward_2.txt", "cubes_formalstudy_upward_3.txt"],
    "practice_cube": ["cubes_practice_cube.txt"],
    "practice_forward": ["cubes_pilotmapforward.txt"],
    "practice_right": ["cubes_pilotmapright.txt"],
    "practice_upward": ["cubes_pilotmapupward.txt"],
}


'''
match the fly_mode with the filename, and copy the flymode file to the cubes.txt
'''
def load_fly_map(fly_map):
    with open(cubes_path + "/" + cubes_filename, "w") as f:
        with open(cubes_path + "/" + fly_map, "r") as f2:
            f.write(f2.read())


def start_new_task(settings, position, orientation):
    print("start new task!")
    command = ''
    for key, value in settings.items():
        if key == "control_mode":
            command += f" --control_mode {value}"
        elif key == "fly_mode":
            command += f" --fly_mode {value}"
            fly_mode = value
        elif key == "participant_name":
            command += f" --participant_name {value}"
        elif key == "is_feedback_on":
            if value == "True":
                command += " --is_feedback_on"
        elif key == "is_assistance_on":
            if value == "True":
                command += " --is_assistance_on"
        elif key == "fly_map":
            fly_map = settings["fly_map"]
            load_fly_map(fly_map)
            command += f" --fly_map {fly_map}"
        else:
            print("bug!!! ", key, value)
            exit(-1)
    command += f" --position {position[0]} {position[1]} {position[2]}"
    command += f" --orientation {orientation[0]} {orientation[1]} {orientation[2]} {orientation[3]}"
    subprocess.Popen(f'start ../TestSceneBright/WindowsNoEditor/Blocks.exe', shell=True)
    time.sleep(3)  # Wait for 1 second
    print(command)
    subprocess.Popen('start cmd /k python airsim_replay.py'+command, shell=True)


'''
data_20240227_174445_formalp12_body_forward_False_False.txt
data_20240227_174650_formalp12_body_right_False_False.txt
data_20240227_174816_formalp12_body_upward_False_False.txt
data_20240227_180119_formalp12_hand_forward_True_False.txt
data_20240227_180445_formalp12_hand_right_True_False.txt
data_20240227_180703_formalp12_hand_upward_True_False.txt
data_20240227_181201_formalp12_hand_forward_True_True.txt
data_20240227_181447_formalp12_hand_right_True_True.txt
data_20240227_181602_formalp12_hand_upward_True_True.txt
data_20240227_182712_formalp12_body_forward_True_False.txt
data_20240227_182937_formalp12_body_right_True_False.txt
data_20240227_183211_formalp12_body_upward_True_False.txt
data_20240227_183833_formalp12_body_forward_True_True.txt
data_20240227_184016_formalp12_body_right_True_True.txt
data_20240227_184143_formalp12_body_upward_True_True.txt
'''

data_filename = "results/" + "data_20240221_103022_formalp1_body_upward_False_False.txt"

# read the first line as a json file, and save the keys and values
# and then find the line with key "situation_awareness", save the "position" and "orientation" from previous line
with open(data_filename, "r") as f:
    line = f.readline()
    settings = json.loads(line)
    position = None
    orientation = None
    last_line = None
    while True:
        line = f.readline()
        if not line:
            break
        data = json.loads(line)
        if "situation_awareness" in data:
            print("situation_awareness: ", data["situation_awareness"])
            last_data = json.loads(last_line)
            position = np.round(last_data["position"], 3)
            orientation = np.round(last_data["orientation"])
            break
        else:
            last_line = line

print("position: ", position, "orientation: ", orientation)
print("settings: ", settings)

start_new_task(settings, position, orientation)