import airsim
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
import argparse
from situation_awareness_popup import get_situation_awareness_answers
np.set_printoptions(precision=3, suppress=True)

###### study configs

def parse_arguments():
    parser = argparse.ArgumentParser(description='Process some parameters.')

    # Define the parameters with their default values
    parser.add_argument('--fly_mode', type=str, default='forward',
                        help='fly mode: forward, right, upward')
    parser.add_argument('--fly_map', type=str, default='none',
                        help='fly map filename')
    parser.add_argument('--control_mode', type=str, default='hand', choices=['hand', 'body'],
                        help='Control mode: "hand" or "body"')
    parser.add_argument('--participant_name', type=str, default='test_p1',
                        help='participant id: test_p1')
    parser.add_argument('--is_feedback_on', action='store_true',
                        help='Flag to turn feedback on. Default is False.')
    parser.add_argument('--is_assistance_on', action='store_true',
                        help='Flag to turn assistance on. Default is False.')
    parser.add_argument('--export_data', action='store_true',
                        help='Flag to export data. Default is False.')
    parser.add_argument('--position', type=float, nargs=3, default=[0, 0, 0],
                        help='Initial position of the drone')
    parser.add_argument('--orientation', type=float, nargs=4, default=[0, 0, 0, 1],
                        help='Initial orientation of the drone')

    # Parse the arguments
    args = parser.parse_args()
    return args

args = parse_arguments()
fly_mode = args.fly_mode
fly_map = args.fly_map
control_mode = args.control_mode
is_feedback_on = args.is_feedback_on
is_assistance_on = args.is_assistance_on
export_data = args.export_data
participant_name = args.participant_name
position = args.position
orientation = args.orientation

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
# client.takeoffAsync().join()


gameController = XboxController()
if not gameController.initSuccess:
    exit(-1)




###### load environment obstacles

filepath = 'D:/2023Fall/DroneSimulation/TestSceneBright/WindowsNoEditor/Blocks/Content/Settings/cubes.txt'

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
        obstacles.append(SafetyConstraint3D(cube['ScaleX']+0.717, cube['ScaleY']+0.717, cube['ScaleZ']+0.717, cube['LocationX']/100, cube['LocationY']/100, cube['LocationZ']/100, 4, 1))
    elif cube["Type"] == "Sphere":
        obstacles.append(SafetyConstraint3D(cube['ScaleX']+0.717, cube['ScaleY']+0.717, cube['ScaleZ']+0.717, cube['LocationX']/100, cube['LocationY']/100, cube['LocationZ']/100, 2, 1))
    elif cube["Type"] == "Cylinder":
        obstacles.append(SafetyConstraint2D(cube['ScaleX']+0.717, cube['ScaleY']+0.717, cube['LocationX']/100, cube['LocationY']/100, 2, 1))
    else:
        print("wrong obstacle type", cube["Type"])
ecbf = ExponentialControlBarrierFunction(obstacles)


# set obstacle IDs so taat we can identify the collisions in data
obs_names = client.simListSceneObjects("StaticMeshActor_.*")
for i, obs_name in enumerate(obs_names):
    client.simSetSegmentationObjectID(obs_name, 10+i, False)


# fly to target position
print(f"Fly to {position} with orientation {orientation}")
# New position (x, y, z) and orientation (pitch, roll, yaw) in radians
new_position = airsim.Vector3r(position[0], position[1], -position[2])  # Example position
new_orientation = airsim.Quaternionr(orientation[0], orientation[1], orientation[2], orientation[3])  # Example orientation (flat and level)
# Create the pose object with the new position and orientation
new_pose = airsim.Pose(new_position, new_orientation)
# Teleport the drone to the new pose
client.simSetVehiclePose(new_pose, True)
client.hoverAsync().join()

time.sleep(1)

###### temp variables
count = 0
start_time = time.time()

while True:
    try:        
        # record the frame start time
        frame_start_time = time.time()
        # read user input from controller
        user_input = gameController.get_controller_input()
        v_ref = np.zeros(3, dtype=np.float64)
        u_ref = np.zeros(6, dtype=np.float64)

        ### xbox controller
        v_rot = np.round(user_input['x_axis'], 3)
        v_ref[0] = -np.round(user_input['w_axis'], 3) * 5
        v_ref[1] = np.round(user_input['z_axis'], 3) * 5
        v_ref[2] = -np.round(user_input['y_axis'], 3) * 5

        ### read drone state from AirSim
        # api_time = time.time()
        state = client.getMultirotorState()
        collision = client.simGetCollisionInfo()
        pos = state.kinematics_estimated.position
        vel = state.kinematics_estimated.linear_velocity
        ori = state.kinematics_estimated.orientation
        # pos_gt = client.simGetGroundTruthKinematics("Drone1").position
        # pos_vh = client.simGetVehiclePose("Drone1").position
        x_ref = np.array([pos.x_val, pos.y_val, -pos.z_val, vel.x_val, vel.y_val, -vel.z_val])
        # api_time = time.time() - api_time
        # print("x_ref = ", x_ref)

        ### convert the user input to drone orientation
        rotation = R.from_quat([0, 0, ori.z_val, ori.w_val])
        v_ref_rotated = rotation.apply(v_ref)
        # calculate diff between v_ref and v_cur, convert into acc command
        a_ref = v_ref_rotated - x_ref[3:6]
        u_ref[3:6] = np.round(a_ref, 3)

        ### use ECBF to find safe input
        # ecbf_time = time.time()
        u_safe, success = ecbf.control_input_optimization(x_ref, u_ref)
        u_safe = np.round(u_safe, 3)
        # ecbf_time = time.time() - ecbf_time
        # u_safe = np.round(u_safe, 3)
        # print("u_ref = ", u_ref)
        # print("u_safe = ", u_safe)
        # x_dot = ecbf.f(x_ref) + ecbf.g(x_ref) @ u_safe
        # x_dot[0:3] = x_dot[0:3] + x_dot[3:6] * delta_time
        # x_safe = x_ref + x_dot * delta_time 
        # x_safe = np.round(x_safe, 3)
        x_safe = x_ref + u_ref
        # print(x_safe.shape, x_ref.shape, u_safe.shape)
        # print("x_safe = ", x_safe, "\n")

        ### update AirSim drone state
        # if np.abs(v_rot) > 0.3:
            # client.moveByAngleRatesZAsync(0, 0, -v_rot, z = -x_ref[2], duration=0.1)
            # client.moveByAngleRatesThrottleAsync(0, 0, -v_rot, throttle = 0.5945, duration=0.1)
        if (np.linalg.norm(x_safe[3:6]) > 1e-2 or np.abs(v_rot) > 0.3):
            client.moveByVelocityAsync(x_safe[3], x_safe[4], -x_safe[5], duration=0.01, yaw_mode=airsim.YawMode(True, v_rot*60)).join()
        else:
            # print("idle")
            client.moveByVelocityAsync(0, 0, 0, duration=0.01).join()
        # client.moveByRollPitchYawZAsync(0, 0, -v_rot, z = -x_safe[], duration=0.01).join()
        # client.moveByAngleRatesThrottleAsync(-v_rot, 0, 0, throttle=1.0, duration=0.1).join()
        # client.rotateByYawRateAsync(-v_rot, duration=0.1)
        # client.moveByVelocityZAsync(x_safe[3], x_safe[4], -x_safe[2], duration=1.0)
        # client.moveToPositionAsync(x_safe[0], x_safe[1], -x_safe[2], 1.0)
        # time.sleep(delta_time

        ### debug output
        count += 1
        current_time = time.time()  # Get the current time
        if current_time - start_time > 1.0: 
            # print("FPS = ", count, ", api = ", np.round(api_time, 4), ", ECBF = ", np.round(ecbf_time, 4), ", ble = ", np.round(ble_time, 4))
            print("pos = ", pos)
            # print("pos_gt = ", pos_gt)
            # print("pos_vh = ", pos_vh)
            print("ori = ", ori)
            count = 0
            start_time = current_time 
            # print("collison = ", collision.has_collided)
            # print("v_ref = ", v_ref)
            # print("v_rot = ", v_ref_rotated)
            # print("v_rot = ", v_rot)
            # print("success = ", success)
            # print("u_ref = ", u_ref)
            # print("u_safe = ", u_safe)
            # print("u_diff = ", u_diff_rot)
            # print("x_safe = ", x_safe, "\n")
            # print("ref safety = ", ecbf.safety_constraint_list[0].safety_constraint(u_ref, x_ref))
            # print("alt safety = ", ecbf.safety_constraint_list[0].safety_constraint(u_safe, x_ref))
            # for i in range(len(ecbf.safety_constraint_list)):
            #     print(i, " ", ecbf.safety_constraint_list[i].safety_constraint(u_ref, x_ref))

        while time.time() < frame_start_time + 0.02:
            continue
    
    except KeyboardInterrupt:
        break