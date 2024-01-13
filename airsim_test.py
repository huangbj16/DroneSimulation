import airsim
import os
import numpy as np
from controller_input import XboxController
from airsim_optimize_exponential import ExponentialControlBarrierFunction, SafetyConstraint2D
from matplotlib.patches import Circle
import matplotlib.pyplot as plt
import time

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Async methods returns Future. Call join() to wait for task to complete.

client.takeoffAsync().join()
client.moveToPositionAsync(0, 0, -5, 2).join()
test_array = []
for i in range(10):
    k = client.getMultirotorState().kinematics_estimated
    print(k.position, k.linear_velocity, k.linear_acceleration)
    test_array.append(k.position.z_val)
print(np.mean(test_array), np.std(test_array))

gameController = XboxController()
if not gameController.initSuccess:
    exit(-1)

epsd1 = SafetyConstraint2D(1.0, 1.0, 10.0, 2.0, 2.0, 9.0)
epsd2 = SafetyConstraint2D(1.0, 1.0, 20.0, -4.0, 2.0, 9.0)
epsd3 = SafetyConstraint2D(1.0, 1.0, 30.0, 2.0, 2.0, 4.0)
epsd4 = SafetyConstraint2D(1.0, 1.0, 40.0, -1.0, 2.0, 4.0)
ecbf = ExponentialControlBarrierFunction([epsd1, epsd2, epsd3, epsd4])

delta_time = 0.1

path = []
count = 0
start_time = time.time()

while True:
    try:
        # read user input from controller
        user_input = gameController.get_controller_input()
        u_ref = np.zeros(6, dtype=np.float32)
        # u_ref[3] = 5.0
        u_ref[3] = -np.round(user_input['w_axis'], 3) * 20
        u_ref[4] = np.round(user_input['z_axis'], 3) * 20
        u_ref[5] = np.round(user_input['y_axis'], 3) * 20 + 5
        # print("u_ref = ", u_ref)

        # read drone state from AirSim
        pos = client.getMultirotorState().kinematics_estimated.position
        vel = client.getMultirotorState().kinematics_estimated.linear_velocity
        x_ref = np.array([pos.x_val, pos.y_val, pos.z_val, vel.x_val, vel.y_val, vel.z_val])

        # use ECBF to find safe input
        u_safe = ecbf.control_input_optimization(x_ref, u_ref)
        u_safe = np.round(u_safe, 3)
        # print("u_safe = ", u_safe)
        x_safe = x_ref + u_safe * delta_time
        x_safe = np.round(x_safe, 3)
        path.append(x_safe[:2])
        # print("x_safe = ", x_safe)

        # update AirSim drone state
        client.moveByVelocityAsync(x_safe[3], x_safe[4], x_safe[5], duration=delta_time)
        # client.moveByVelocityZAsync(x_safe[3], x_safe[4], x_safe[2], duration=delta_time)
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
        circles = [Circle((10, 2), 3, color='blue', fill=False),
            Circle((20, -4), 3, color='blue', fill=False),
            Circle((30, 2), 2, color='blue', fill=False),
            Circle((40, -1), 2, color='blue', fill=False)
        ]
        for circle in circles:
            plt.gca().add_patch(circle)
        plt.show()