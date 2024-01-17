import airsim
import os
import numpy as np
from controller_input import XboxController
from airsim_optimize_exponential import ExponentialControlBarrierFunction, SafetyConstraint2D, SafetyConstraint3D, SafetyConstraintWall3D
from matplotlib.patches import Circle, Rectangle
import matplotlib.pyplot as plt
import time
import json

np.set_printoptions(precision=3, suppress=False)


# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Async methods returns Future. Call join() to wait for task to complete.

client.takeoffAsync().join()
client.moveToPositionAsync(0, 0, -3, 1)
# time.sleep(1)

test_array = []
for i in range(10):
    k = client.getMultirotorState().kinematics_estimated
    # print(k.position, k.linear_velocity, k.linear_acceleration)
    test_array.append(k.position.z_val)
# print(np.mean(test_array), np.std(test_array))

# connect to game controller
gameController = XboxController()
if not gameController.initSuccess:
    exit(-1)


# load obstacle settings

filepath = 'D:/2023Fall/DroneSimulation/TestScene/WindowsNoEditor/Blocks/Content/Settings/cubes.txt'

def read_cubes_file(file_path):
    cubes = []
    with open(file_path, 'r') as file:
        for line in file:
            cubes.append(json.loads(line.strip()))
    return cubes

cubes_data = read_cubes_file(filepath)

def get_forward_vector(roll, pitch, yaw):
    forward = np.array([0, 0, 1], dtype=np.float32)
    if roll != 0:
        roll_radians = np.radians(roll)
        rotation_roll = np.array([[1, 0, 0],
                                [0, np.cos(roll_radians), -np.sin(roll_radians)],
                                [0, np.sin(roll_radians), np.cos(roll_radians)]])
        forward = rotation_roll.dot(forward)
    if pitch != 0:
        pitch_radians = np.radians(pitch)
        rotation_pitch = np.array([[np.cos(pitch_radians), 0, np.sin(pitch_radians)],
                            [0, 1, 0],
                            [-np.sin(pitch_radians), 0, np.cos(pitch_radians)]])
        forward = rotation_pitch.dot(forward)
    if yaw != 0:
        yaw_radians = np.radians(yaw)
        rotation_yaw = np.array([[np.cos(yaw_radians), -np.sin(yaw_radians), 0],
                              [np.sin(yaw_radians), np.cos(yaw_radians), 0],
                              [0, 0, 1]])
        forward = rotation_yaw.dot(forward)
    return np.round(forward, 3)

obstacles = []
for cube in cubes_data:
    if cube["Type"] == "Plane":
        forward_vector = get_forward_vector(-cube["RotationRoll"], -cube["RotationPitch"], cube["RotationYaw"])
        print("forward = ", forward_vector)
        obstacles.append(SafetyConstraintWall3D(forward_vector[0], forward_vector[1], forward_vector[2], cube['LocationX']/100+forward_vector[0], cube['LocationY']/100+forward_vector[1], cube['LocationZ']/100+forward_vector[2]))
    elif cube["Type"] == "Cube":
        obstacles.append(SafetyConstraint3D(cube['ScaleX'], cube['ScaleY'], cube['ScaleZ'], cube['LocationX']/100, cube['LocationY']/100, cube['LocationZ']/100, 4, 16))
    elif cube["Type"] == "Sphere":
        obstacles.append(SafetyConstraint3D(cube['ScaleX'], cube['ScaleY'], cube['ScaleZ'], cube['LocationX']/100, cube['LocationY']/100, cube['LocationZ']/100, 2, 4))
    elif cube["Type"] == "Cylinder":
        obstacles.append(SafetyConstraint2D(cube['ScaleX'], cube['ScaleY'], cube['LocationX']/100, cube['LocationY']/100, 2, 4))
    else:
        print("wrong obstacle type", cube["Type"])
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
        u_ref[3] = -np.round(user_input['w_axis'], 3) * 10
        u_ref[4] = np.round(user_input['z_axis'], 3) * 10
        u_ref[5] = -np.round(user_input['y_axis'], 3) * 10 - 3

        # read drone state from AirSim
        pos = client.getMultirotorState().kinematics_estimated.position
        vel = client.getMultirotorState().kinematics_estimated.linear_velocity
        x_ref = np.array([pos.x_val, pos.y_val, -pos.z_val, vel.x_val, vel.y_val, -vel.z_val])
        # print("x_ref = ", x_ref)

        # use ECBF to find safe input
        u_safe, success = ecbf.control_input_optimization(x_ref, u_ref)
        u_safe = np.round(u_safe, 3)
        # print("u_ref = ", u_ref)
        # print("u_safe = ", u_safe)
        # x_dot = ecbf.f(x_ref) + ecbf.g(x_ref) @ u_safe
        # x_dot[0:3] = x_dot[0:3] + x_dot[3:6] * delta_time
        # x_safe = x_ref + x_dot * delta_time 
        # x_safe = np.round(x_safe, 3)
        x_safe = x_ref + u_safe * delta_time
        path.append(x_safe[:2])
        # print("x_safe = ", x_safe, "\n")

        # update AirSim drone state
        client.moveByVelocityAsync(x_safe[3], x_safe[4], -x_safe[5], duration=1.0)
        # client.moveByVelocityZAsync(x_safe[3], x_safe[4], -x_safe[2], duration=1.0)
        # client.moveToPositionAsync(x_safe[0], x_safe[1], -x_safe[2], 1.0)
        # time.sleep(delta_time)
        count += 1

        current_time = time.time()  # Get the current time
        if current_time - start_time > 1: 
            print("FPS = ", count)
            count = 0
            start_time = current_time
            print("x_ref = ", x_ref)
            print("u_ref = ", u_ref)
            print("u_safe = ", u_safe)
            print("x_safe = ", x_safe, "\n")
            print("success = ", success)
            print("ref safety = ", ecbf.safety_constraint_list[0].safety_constraint(u_ref, x_ref))
            print("alt safety = ", ecbf.safety_constraint_list[0].safety_constraint(u_safe, x_ref))
    
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