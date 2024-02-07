# Python 3
import socket
import json

class FalconSocketClient:
    def __init__(self) -> None:
        host = '127.0.0.1'
        port = 65432
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        try:
            self.client.connect((host, port))
            print("Connection successful.")
            self.initSuccess = True
        except socket.error as e:
            print(f"Connection failed with error: {e}")
            self.initSuccess = False

    def get_controller_input(self):
        command = {'command': 'get_joystick_position'}
        command = json.dumps(command).encode('utf-8')
        self.client.sendall(command)
        response = self.client.recv(1024)
        # print('Received', response.decode('utf-8'))
        response = json.loads(response.decode('utf-8'))
        position = response['joystick_position']
        return position

    def set_force(self, value):
        command = {'command': 'set_force', 'value': value}
        command =  json.dumps(command).encode('utf-8')
        self.client.sendall(command)
        response = self.client.recv(1024)
        # print('Received', response.decode('utf-8'))

    def get_button_state(self):
        command = {'command': 'get_button_state'}
        command = json.dumps(command).encode('utf-8')
        self.client.sendall(command)
        response = self.client.recv(1024)
        # print('Received', response.decode('utf-8'))
        response = json.loads(response.decode('utf-8'))
        button = response['button_state']
        return button

if __name__ == '__main__':
    # Example usage
    client = FalconSocketClient()
    client.set_force([5, 0, 0])
    while True:
        client.get_joystick_position()
