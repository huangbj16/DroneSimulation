import pygame
import pygame_gui


def get_situation_awareness_answers() -> dict:

    # Initialize pygame
    # pygame.init()

    # Set the size of the main window
    window_size = (500, 500)
    main_window = pygame.display.set_mode(window_size)

    # Set up the GUI manager for Pygame GUI
    manager = pygame_gui.UIManager(window_size)

    # Dictionary to store the answers
    answers_dict = {}

    # Function to create the pop-up window
    def create_popup():
        # Create a window
        # popup_window = pygame_gui.elements.UIWindow(rect=pygame.Rect((250, 200), (300, 200)), manager=manager, window_display_title="Questionnaire")

        # Add text labels for questions
        question1 = pygame_gui.elements.UILabel(relative_rect=pygame.Rect(10, 10, 400, 40), text="Question 1: where are the obstacles?", manager=manager)
        question2 = pygame_gui.elements.UILabel(relative_rect=pygame.Rect(10, 210, 400, 40), text="Question 2: how to avoid the obstacles?", manager=manager)

        # Add text entry boxes for answers
        answer1_entry = pygame_gui.elements.UITextEntryBox(relative_rect=pygame.Rect(10, 50, 400, 100), manager=manager)
        answer2_entry = pygame_gui.elements.UITextEntryBox(relative_rect=pygame.Rect(10, 250, 400, 100), manager=manager)

        # Add a submit button
        submit_btn = pygame_gui.elements.UIButton(relative_rect=pygame.Rect(10, 400, 400, 50), text="Submit", manager=manager)

        return answer1_entry, answer2_entry, submit_btn

    # Create the popup and get the elements
    answer1_entry, answer2_entry, submit_btn = create_popup()

    clock = pygame.time.Clock()
    is_running = True

    while is_running:
        time_delta = clock.tick(60)/1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                is_running = False

            if event.type == pygame_gui.UI_BUTTON_PRESSED:
                if event.ui_element == submit_btn:
                    # Store the answers in the dictionary
                    answers_dict['Question1'] = answer1_entry.get_text()
                    answers_dict['Question2'] = answer2_entry.get_text()
                    # Close the window
                    is_running = False

            manager.process_events(event)

        manager.update(time_delta)

        main_window.fill((127, 127, 127))
        manager.draw_ui(main_window)

        pygame.display.update()

    # Print the answers to check
    print(answers_dict)

    # pygame.quit()

    return answers_dict
