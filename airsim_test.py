import airsim
import os
import numpy as np
from controller_input import XboxController
from test_socket_client import FalconSocketClient
from airsim_optimize_exponential import ExponentialControlBarrierFunction, SafetyConstraint2D, SafetyConstraint3D, SafetyConstraintWall3D
from matplotlib.patches import Circle, Rectangle
import matplotlib.pyplot as plt
import time
import json
from scipy.spatial.transform import Rotation as R
from datetime import datetime
from evaluation_module import EvaluationModule
from tactile_feedback_module import TactileFeedbackModule

np.set_printoptions(precision=3, suppress=True)

control_mode = "body"

if control_mode == "body":
    tactile_module = TactileFeedbackModule()

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
    # print(k.position, k.linear_velocity, k.linear_acceleration)f
    test_array.append(k.position.z_val)
# print(np.mean(test_array), np.std(test_array))

### connect to game controller or falcon controller
if control_mode == "body":
    ### xbox controller
    gameController = XboxController()
elif control_mode == "hand":
    ## falcon controller
    gameController = FalconSocketClient()
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
    forward = np.array([0, 0, 1], dtype=np.float64)
    ort1 = np.array([1, 0, 0], dtype=np.float64)
    ort2 = np.array([0, 1, 0], dtype=np.float64)
    if roll != 0:
        roll_radians = np.radians(roll)
        rotation_roll = np.array([[1, 0, 0],
                                [0, np.cos(roll_radians), -np.sin(roll_radians)],
                                [0, np.sin(roll_radians), np.cos(roll_radians)]])
        forward = rotation_roll.dot(forward)
        ort1 = rotation_roll.dot(ort1)
        ort2 = rotation_roll.dot(ort2)
    if pitch != 0:
        pitch_radians = np.radians(pitch)
        rotation_pitch = np.array([[np.cos(pitch_radians), 0, np.sin(pitch_radians)],
                            [0, 1, 0],
                            [-np.sin(pitch_radians), 0, np.cos(pitch_radians)]])
        forward = rotation_pitch.dot(forward)
        ort1 = rotation_pitch.dot(ort1)
        ort2 = rotation_pitch.dot(ort2)
    if yaw != 0:
        yaw_radians = np.radians(yaw)
        rotation_yaw = np.array([[np.cos(yaw_radians), -np.sin(yaw_radians), 0],
                              [np.sin(yaw_radians), np.cos(yaw_radians), 0],
                              [0, 0, 1]])
        forward = rotation_yaw.dot(forward)
        ort1 = rotation_yaw.dot(ort1)
        ort2 = rotation_yaw.dot(ort2)
    return np.round(forward, 3), np.round(ort1, 3), np.round(ort2, 3)

obstacles = []
for cube in cubes_data:
    if cube["Type"] == "Plane":
        forward_vector, ort1, ort2 = get_forward_vector(-cube["RotationRoll"], -cube["RotationPitch"], cube["RotationYaw"])
        print("forward = ", forward_vector)
        print("ort1 = ", ort1)
        print("ort2 = ", ort2)
        obstacles.append(SafetyConstraintWall3D(forward_vector[0], forward_vector[1], forward_vector[2], cube['LocationX']/100+forward_vector[0], cube['LocationY']/100+forward_vector[1], cube['LocationZ']/100+forward_vector[2], cube['ScaleX'], cube['ScaleY'], cube['ScaleZ'], ort1, ort2))
    elif cube["Type"] == "Cube":
        obstacles.append(SafetyConstraint3D(cube['ScaleX'], cube['ScaleY'], cube['ScaleZ'], cube['LocationX']/100, cube['LocationY']/100, cube['LocationZ']/100, 4, 16))
    elif cube["Type"] == "Sphere":
        obstacles.append(SafetyConstraint3D(cube['ScaleX'], cube['ScaleY'], cube['ScaleZ'], cube['LocationX']/100, cube['LocationY']/100, cube['LocationZ']/100, 2, 4))
    elif cube["Type"] == "Cylinder":
        obstacles.append(SafetyConstraint2D(cube['ScaleX'], cube['ScaleY'], cube['LocationX']/100, cube['LocationY']/100, 2, 4))
    else:
        print("wrong obstacle type", cube["Type"])
ecbf = ExponentialControlBarrierFunction(obstacles)

evaluation_module = EvaluationModule()

### set obstacle IDs
obs_names = client.simListSceneObjects("StaticMeshActor_.*")
for i, obs_name in enumerate(obs_names):
    client.simSetSegmentationObjectID(obs_name, 10+i, False)

delta_time = 0.1

path = []
count = 0
start_time = time.time()

sc_values = [[] for _ in range(len(obstacles))]

while True:
    try:
        # read user input from controller
        user_input = gameController.get_controller_input()
        # user_input = {"x_axis": 0.0, "y_axis": 0.0, "z_axis": 0.0, "w_axis": -0.5}
        # print(user_input)
        v_ref = np.zeros(3, dtype=np.float64)
        u_ref = np.zeros(6, dtype=np.float64)
        # u_ref[3] = 20.0

        if control_mode == "body":
            ### xbox controller
            v_rot = np.round(user_input['x_axis'], 3)
            v_ref[0] = -np.round(user_input['w_axis'], 3) * 10
            v_ref[1] = np.round(user_input['z_axis'], 3) * 10
            v_ref[2] = -np.round(user_input['y_axis'], 3) * 10
        elif control_mode == "hand":
            ### falcon controller
            button_val = gameController.get_button_state()
            v_rot = 0
            if (button_val & 2) == 2:
                v_rot = -1.0
            if (button_val & 8) == 8:
                v_rot = 1.0
            v_ref[0] = -np.round(user_input[2], 3) * 200
            v_ref[1] = np.round(user_input[0], 3) * 200
            v_ref[2] = np.round(user_input[1], 3) * 200

        ### read drone state from AirSim
        state = client.getMultirotorState()
        pos = state.kinematics_estimated.position
        vel = state.kinematics_estimated.linear_velocity
        ori = state.kinematics_estimated.orientation
        x_ref = np.array([pos.x_val, pos.y_val, -pos.z_val, vel.x_val, vel.y_val, -vel.z_val])
        # print("x_ref = ", x_ref)

        ### convert the user input to drone orientation
        rotation = R.from_quat([0, 0, ori.z_val, ori.w_val])
        v_ref_rotated = rotation.apply(v_ref)
        # calculate diff between v_ref and v_cur, convert into acc command
        a_ref = v_ref_rotated - x_ref[3:6]
        u_ref[3:6] = a_ref

        ### use ECBF to find safe input
        ecbf_time = time.time()
        u_safe, success = ecbf.control_input_optimization(x_ref, u_ref)
        ecbf_time = time.time() - ecbf_time
        # u_safe = np.round(u_safe, 3)
        # print("u_ref = ", u_ref)
        # print("u_safe = ", u_safe)
        # x_dot = ecbf.f(x_ref) + ecbf.g(x_ref) @ u_safe
        # x_dot[0:3] = x_dot[0:3] + x_dot[3:6] * delta_time
        # x_safe = x_ref + x_dot * delta_time 
        # x_safe = np.round(x_safe, 3)
        x_safe = x_ref + u_safe
        # print(x_safe.shape, x_ref.shape, u_safe.shape)
        # print("x_safe = ", x_safe, "\n")

        ### update AirSim drone state
        # if np.abs(v_rot) > 0.3:
            # client.moveByAngleRatesZAsync(0, 0, -v_rot, z = -x_ref[2], duration=0.1)
            # client.moveByAngleRatesThrottleAsync(0, 0, -v_rot, throttle = 0.5945, duration=0.1)
        if np.linalg.norm(x_safe[3:6]) > 1e-2 or np.abs(v_rot) > 0.3:
            # print("control")
            client.moveByVelocityAsync(x_safe[3], x_safe[4], -x_safe[5], duration=0.01, yaw_mode=airsim.YawMode(True, v_rot*60))
        else:
            # print("idle")
            client.moveByVelocityAsync(0, 0, 0, duration=0.01)
        # client.moveByRollPitchYawZAsync(0, 0, -v_rot, z = -x_safe[], duration=0.01).join()
        # client.moveByAngleRatesThrottleAsync(-v_rot, 0, 0, throttle=1.0, duration=0.1).join()
        # client.rotateByYawRateAsync(-v_rot, duration=0.1)
        # client.moveByVelocityZAsync(x_safe[3], x_safe[4], -x_safe[2], duration=1.0)
        # client.moveToPositionAsync(x_safe[0], x_safe[1], -x_safe[2], 1.0)
        # time.sleep(delta_time
            
        ### apply force feedback
        u_diff = u_safe[3:6] - u_ref[3:6]
        u_diff_rot = rotation.inv().apply(u_diff)
        if control_mode == "hand":
            ### falcon controller
            gameController.set_force([u_diff_rot[1], u_diff_rot[2], -u_diff_rot[0]])
        elif control_mode == "body":
            ### xbox controller, tactile feedback
            if not np.allclose(u_ref, u_safe, rtol=1e-05, atol=1e-08): # input is modified
                for i in range(len(obstacles)):
                    obs = obstacles[i]
                    if obs.isInRange(x_ref):
                        sc_val = obs.safety_constraint(u_ref)
                        if sc_val < 0: # obstacle affected
                            if type(obs) == SafetyConstraint3D:
                                relative_direction = np.array([obs.d1, obs.d2, obs.d3]) - x_ref[0:3]
                                duty = np.power(np.abs(sc_val), 1/obs.n)
                            elif type(obs) == SafetyConstraint2D:
                                relative_direction = np.array([obs.d1, obs.d2, 0]) - np.array([x_ref[0], x_ref[1], 0])
                                duty = np.power(np.abs(sc_val), 1/obs.n)
                            elif type(obs) == SafetyConstraintWall3D:
                                relative_direction = np.array([-obs.a1, -obs.a2, -obs.a3])
                                duty = np.abs(sc_val)
                            else:
                                exit(-1)
                            sc_values[i].append(duty)
                            duty = int(np.clip(duty, 0, 15))
                            # print(i, type(obs), relative_direction)
                            relative_direction = rotation.inv().apply(relative_direction) # rotate to body frame
                            relative_direction /= np.linalg.norm(relative_direction)
                            angle_distances = np.dot(tactile_module.motor_directions, relative_direction)
                            # nearest_index = np.argmax(angle_distances)
                            nearest_indices = np.where(angle_distances == np.max(angle_distances))[0]
                            # print(nearest_indices)
                            for nearest_index in nearest_indices:
                                tactile_module.set_vibration(tactile_module.motor_ids[nearest_index], duty)
                tactile_module.flush_update()
                        
        
        ### save data frame
        collision = client.simGetCollisionInfo()
        data = {
            # "timestamp": datetime.now().timestamp(), 
            "timestamp": state.timestamp,
            "position": x_ref[0:3].tolist(),
            "velocity": x_ref[3:6].tolist(),
            "collision": {
                "has_collided": collision.has_collided,
                "position": [collision.position.x_val, collision.position.y_val, -collision.position.z_val],
                "object_id": collision.object_id,
                "object_name": collision.object_name,
                "time_stamp": collision.time_stamp,
                },
            "user_input": u_ref[3:6].tolist(),
            "safe_input": u_safe[3:6].tolist(),
            "input_diff": u_diff_rot.tolist()
            }
        evaluation_module.frame_update(data)

        ### debug output
        count += 1
        current_time = time.time()  # Get the current time
        if current_time - start_time > 1.0: 
            print("FPS = ", count, ", ECBF = ", ecbf_time)
            # print("ori = ", ori)
            count = 0
            start_time = current_time 
            # print("x_ref = ", x_ref)
            # print("v_ref = ", v_ref)
            # print("v_rot = ", v_rot)
            # print("u_ref = ", u_ref)
            # print("u_safe = ", u_safe)
            # print("x_safe = ", x_safe, "\n")
            # print("ref safety = ", ecbf.safety_constraint_list[0].safety_constraint(u_ref, x_ref))
            # print("alt safety = ", ecbf.safety_constraint_list[0].safety_constraint(u_safe, x_ref))
            # for i in range(len(ecbf.safety_constraint_list)):
            #     print(i, " ", ecbf.safety_constraint_list[i].safety_constraint(u_ref, x_ref))
    
    except KeyboardInterrupt:
        # evaluation_module.export_data()
        for i, sc_value in enumerate(sc_values):
            if len(sc_value) != 0:
                print(i, type(obstacles[i]), np.min(sc_value), np.max(sc_value), np.mean(sc_value), np.std(sc_value))
        break