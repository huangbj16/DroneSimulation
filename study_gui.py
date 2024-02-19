import pygame
import pygame_gui
import json

pygame.init()

# Load settings
with open('study_settings.json', 'r') as f:
    settings = json.load(f)

# Window setup
window_size = (600, 600)
window = pygame.display.set_mode(window_size)
manager = pygame_gui.UIManager(window_size)
pygame.display.set_caption("Settings Configuration")

# Layout configuration
start_x, start_y = 50, 30  # Starting position for the first element
vertical_spacing = 10  # Space between elements
button_height = 30
current_y = start_y

# Function to add vertical space
def add_vertical_space(height):
    global current_y
    current_y += height

setting_names = []
setting_UI_elements = []

# Create UI elements with labels
for key, value in settings.items():
    # Create and render the label for the setting
    setting_names.append(key)
    label = pygame_gui.elements.UILabel(relative_rect=pygame.Rect((start_x, current_y), (200, button_height)),
                                        text=key,
                                        manager=manager)
    current_y += label.rect.height + 5  # Small spacing between label and the input element

    if len(value) > 1:  # For multiple choices, create a dropdown
        options = value
        drop_down = pygame_gui.elements.UIDropDownMenu(options_list=options,
                                                        starting_option=options[0],
                                                        relative_rect=pygame.Rect((start_x + 210, current_y - 30), (200, button_height)),
                                                        manager=manager)
        setting_UI_elements.append(drop_down)
        current_y += drop_down.rect.height + vertical_spacing
    elif value[0] in ['True', 'False']:  # For boolean values, create checkboxes
        options = value
        checkbox = pygame_gui.elements.UIDropDownMenu(options_list=['False', "True"],
                                                        starting_option=options[0],
                                                        relative_rect=pygame.Rect((start_x + 210, current_y - 30), (200, button_height)),
                                                        manager=manager)
        setting_UI_elements.append(checkbox)
        current_y += checkbox.rect.height + vertical_spacing
    else:  # For text inputs, create text entry fields
        text_box = pygame_gui.elements.UITextEntryLine(relative_rect=pygame.Rect((start_x + 210, current_y - 30), (200, button_height)),
                                                       manager=manager)
        setting_UI_elements.append(text_box)
        current_y += text_box.rect.height + vertical_spacing

    add_vertical_space(vertical_spacing)  # Add extra space after each category

start_button = pygame_gui.elements.UIButton(relative_rect=pygame.Rect((start_x, current_y), (200, button_height)),
                                                text="START",
                                                manager=manager)

print(setting_names)
print(setting_UI_elements)

import subprocess
import time

def start_new_task():
    print("start!")
    # Step 1: Run a Python script in a new cmd window
    subprocess.Popen(f'start cmd /k C:/python27/python.exe ./FalconBindings/test_socket_server.py', shell=True)
    time.sleep(3)  # Wait for 1 second
    command = ''
    for key, value in zip(setting_names, setting_UI_elements):
        if key == "control_mode":
            command += f" --control_mode {value.selected_option}"
        elif key == "fly_mode":
            command += f" --fly_mode {value.selected_option}"
        elif key == "participant_id":
            command += f" --participant_name {value.get_text()}"
        elif key == "is_feedback_on":
            if value.selected_option == "True":
                command += " --is_feedback_on"
        elif key == "is_assistance_on":
            if value.selected_option == "True":
                command += " --is_assistance_on"
        elif key == "export_data":
            if value.selected_option == "True":
                command += " --export_data"
        else:
            print("bug!!! ", key, value)
            exit(-1)
    subprocess.Popen(f'start ../TestSceneBright/WindowsNoEditor/Blocks.exe', shell=True)
    time.sleep(3)  # Wait for 1 second
    print(command)
    subprocess.Popen('start cmd /k conda activate airsim && python airsim_test.py'+command, shell=True)


# Main loop
clock = pygame.time.Clock()
is_running = True

while is_running:
    time_delta = clock.tick(60) / 1000.0
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            is_running = False
        if event.type == pygame_gui.UI_BUTTON_PRESSED:
            if event.ui_element == start_button:
                start_new_task()

        
        manager.process_events(event)
    
    manager.update(time_delta)

    window.fill(manager.ui_theme.get_colour('dark_bg'))
    manager.draw_ui(window)

    pygame.display.update()

pygame.quit()
