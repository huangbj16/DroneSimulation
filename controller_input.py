import pygame

class XboxController:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()

        self.joystick = None
        self.initSuccess = self.initialize_joystick()

    def initialize_joystick(self):
        joystick_count = pygame.joystick.get_count()
        print(joystick_count)
        if joystick_count > 0:
            self.joystick = pygame.joystick.Joystick(0)  # Assumes first joystick
            self.joystick.init()
            return True
        else:
            print("No joystick detected")
            return False

    def get_controller_input(self):
        if self.joystick is None:
            return None

        pygame.event.pump()  # Process event queue

        # Assuming two axes for the left joystick
        x_axis = self.joystick.get_axis(0)  # X-axis
        y_axis = self.joystick.get_axis(1)  # Y-axis
        z_axis = self.joystick.get_axis(2)  # X-axis
        w_axis = self.joystick.get_axis(3)  # Y-axis

        return {"x_axis": x_axis, "y_axis": y_axis, "z_axis": z_axis, "w_axis":w_axis}

# Example usage
if __name__ == "__main__":
    controller = XboxController()

    try:
        while True:
            joystick_state = controller.get_controller_input()
            if joystick_state is not None:
                print(f"X-axis: {joystick_state['x_axis']}, Y-axis: {joystick_state['y_axis']}, Z-axis: {joystick_state['z_axis']}, W-axis: {joystick_state['w_axis']}")
            pygame.time.wait(100)  # Wait 100 milliseconds

    except KeyboardInterrupt:
        pygame.quit()
