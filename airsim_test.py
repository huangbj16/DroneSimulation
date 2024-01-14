import airsim
import os
import numpy as np
from controller_input import XboxController
from airsim_optimize_exponential import ExponentialControlBarrierFunction, SafetyConstraint2D, SafetyConstraint3D
from matplotlib.patches import Circle, Rectangle
import matplotlib.pyplot as plt
import time
import json

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Async methods returns Future. Call join() to wait for task to complete.

client.takeoffAsync().join()
# client.moveToPositionAsync(1, 1, -2, 2).join()

test_array = []
for i in range(10):
    k = client.getMultirotorState().kinematics_estimated
    print(k.position, k.linear_velocity, k.linear_acceleration)
    test_array.append(k.position.z_val)
print(np.mean(test_array), np.std(test_array))

# connect to game controller
gameController = XboxController()
if not gameController.initSuccess:
    exit(-1)


# load obstacle settings

path = 'D:/2023Fall/DroneSimulation/TestScene/WindowsNoEditor/Blocks/Content/Settings/cubes.txt'

def read_cubes_file(file_path):
    cubes = []
    with open(file_path, 'r') as file:
        for line in file:
            cubes.append(json.loads(line.strip()))
    return cubes

cubes_data = read_cubes_file(path)

# epsd1 = SafetyConstraint2D(1.0, 1.0, 10.0, 2.0, 2.0, 9.0)
# epsd2 = SafetyConstraint2D(1.0, 1.0, 20.0, -4.0, 2.0, 9.0)
# epsd3 = SafetyConstraint2D(1.0, 1.0, 30.0, 2.0, 2.0, 4.0)
# epsd4 = SafetyConstraint2D(1.0, 1.0, 40.0, -1.0, 2.0, 4.0)
obstacles = []
for cube in cubes_data:
    print(cube)
    obstacles.append(SafetyConstraint3D(1.0, 1.0, 1.0, cube['LocationX']/100, cube['LocationY']/100, cube['LocationZ']/100, 4, 16))
ecbf = ExponentialControlBarrierFunction(obstacles)


delta_time = 0.1

path = []
count = 0
start_time = time.time()

while True:
    try:
        # read user input from controller
        user_input = gameController.get_controller_input()
        u_ref = np.zeros(6, dtype=np.float32)
        # u_ref[3] = 20.0
        u_ref[3] = -np.round(user_input['w_axis'], 3) * 20
        u_ref[4] = np.round(user_input['z_axis'], 3) * 20
        u_ref[5] = -np.round(user_input['y_axis'], 3) * 20 - 3

        # read drone state from AirSim
        pos = client.getMultirotorState().kinematics_estimated.position
        vel = client.getMultirotorState().kinematics_estimated.linear_velocity
        x_ref = np.array([pos.x_val, pos.y_val, -pos.z_val, vel.x_val, vel.y_val, -vel.z_val])
        print("x_ref = ", np.round(x_ref, 3))

        # use ECBF to find safe input
        u_safe = ecbf.control_input_optimization(x_ref, u_ref)
        u_safe = np.round(u_safe, 3)
        print("u_ref = ", u_ref)
        print("u_safe = ", u_safe)
        x_safe = x_ref + u_safe * delta_time
        x_safe = np.round(x_safe, 3)
        path.append(x_safe[:2])
        print("x_safe = ", x_safe, "\n")

        # update AirSim drone state
        client.moveByVelocityAsync(x_safe[3], x_safe[4], -x_safe[5], duration=delta_time)
        # client.moveByVelocityZAsync(x_safe[3], x_safe[4], -x_safe[2], duration=delta_time)
        time.sleep(delta_time)
        count += 1
        current_time = time.time()  # Get the current time
        if current_time - start_time > 1: 
            print("FPS = ", count)
            count = 0
            start_time = current_time
    
    except KeyboardInterrupt: 
        path = np.array(path)
        plt.scatter(path[:, 0], path[:, 1])
        for obs in obstacles:
            print(obs.d1, obs.d2)
            rect = Rectangle((obs.d1-1.0, obs.d2-1.0), 2.0, 2.0, color='blue', fill=False)
            plt.gca().add_patch(rect)
        # circles = [Circle((10, 2), 3, color='blue', fill=False),
        #     Circle((20, -4), 3, color='blue', fill=False),
        #     Circle((30, 2), 2, color='blue', fill=False),
        #     Circle((40, -1), 2, color='blue', fill=False)
        # ]
        # for circle in circles:
        #     plt.gca().add_patch(circle)
        plt.show()