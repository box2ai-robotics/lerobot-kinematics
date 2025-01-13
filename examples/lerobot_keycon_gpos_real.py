# code by LinCC111 2025.1.13 Box2AI-Robotics copyright 盒桥智能 版权所有

import os
import mujoco
import mujoco.viewer
import numpy as np
import time
from lerobot_kinematics import lerobot_IK, lerobot_FK

from pynput import keyboard
import threading

# For Feetech Motors
from feetech import FeetechMotorsBus
import json

np.set_printoptions(linewidth=200)

# Set up the MuJoCo render backend
os.environ["MUJOCO_GL"] = "egl"

# Define joint names
JOINT_NAMES = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"]

# Absolute path of the XML model
xml_path = "./examples/scene.xml"
mjmodel = mujoco.MjModel.from_xml_path(xml_path)
qpos_indices = np.array([mjmodel.jnt_qposadr[mjmodel.joint(name).id] for name in JOINT_NAMES])
mjdata = mujoco.MjData(mjmodel)

# Define joint control increment (in radians)
JOINT_INCREMENT = 0.005
POSITION_INSERMENT = 0.0008

# Define joint limits
qlimit = [[-2.1, -3.14, -0.1, -2.0, -3.1, -0.1],
          [2.1, 0.2, 3.14, 1.8, 3.1, 1.0]]

glimit = [[0.000, -0.4, 0.046, -3.1, -1.5, -1.5],
          [0.430, 0.4, 0.23, 3.1, 1.5, 1.5]]

# Initialize target joint positions
init_qpos = np.array([0.0, -3.14, 3.14, 0.0, -1.57, -0.157])
target_qpos = init_qpos.copy()
init_gpos = lerobot_FK(init_qpos[0:5])
target_gpos = init_gpos.copy()

# Thread-safe lock for key press management
lock = threading.Lock()

# Key mapping for joint control
key_to_joint_increase = {
    'w': 0,  # Move forward
    'a': 1,  # Move right
    'r': 2,  # Move up
    'e': 3,  # Roll +
    't': 4,  # Pitch +
    'z': 5,  # Gripper +
}

key_to_joint_decrease = {
    's': 0,  # Move backward
    'd': 1,  # Move left
    'f': 2,  # Move down
    'q': 3,  # Roll -
    'g': 4,  # Pitch -
    'c': 5,  # Gripper -
}

# Dictionary to track currently pressed keys and their direction
keys_pressed = {}

# Callback for key press events
def on_press(key):
    try:
        k = key.char.lower()  # Convert to lowercase to handle both lowercase and uppercase inputs
        if k in key_to_joint_increase:
            with lock:
                keys_pressed[k] = 1  # Increase direction
        elif k in key_to_joint_decrease:
            with lock:
                keys_pressed[k] = -1  # Decrease direction
        elif k == "0":
            with lock:
                global target_qpos, target_gpos
                target_qpos = init_qpos.copy()  # Reset to initial position
                target_gpos = init_gpos.copy()  # Reset to initial gripper position
        print(f'{key}')

    except AttributeError:
        pass  # Handle special keys if needed

# Callback for key release events
def on_release(key):
    try:
        k = key.char.lower()
        if k in keys_pressed:
            with lock:
                del keys_pressed[k]
    except AttributeError:
        pass  # Handle special keys if needed

# Start the keyboard listener in a separate thread
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

target_gpos_last = init_gpos.copy()

# Connect to the robotic arm motors
motors = {"shoulder_pan": (1, "sts3215"),
          "shoulder_lift": (2, "sts3215"),
          "elbow_flex": (3, "sts3215"),
          "wrist_flex": (4, "sts3215"),
          "wrist_roll": (5, "sts3215"),
          "gripper": (6, "sts3215")}

follower_arm = FeetechMotorsBus(port="/dev/lerobot_tty1", motors=motors)
follower_arm.connect()
print('Robot arm connected successfully')

# Load the robot arm calibration parameters
arm_calib_path = "examples/main_follower.json" 
with open(arm_calib_path) as f:
    calibration = json.load(f)
follower_arm.set_calibration(calibration)

try:
    # Launch MuJoCo viewer
    with mujoco.viewer.launch_passive(mjmodel, mjdata) as viewer:
        start = time.time()
        while viewer.is_running() and time.time() - start < 1000:
            step_start = time.time()

            with lock:
                for k, direction in keys_pressed.items():
                    if k in key_to_joint_increase:
                        position_idx = key_to_joint_increase[k]
                        if position_idx == 1 or position_idx == 5:
                            position_idx = 0 if position_idx == 1 else 5
                            if target_qpos[position_idx] < qlimit[1][position_idx] - JOINT_INCREMENT * direction:
                                target_qpos[position_idx] += JOINT_INCREMENT * direction
                        elif position_idx == 4 or position_idx == 3:
                            if target_gpos[position_idx] <= glimit[1][position_idx]:
                                target_gpos[position_idx] += POSITION_INSERMENT * direction * 4
                        else:
                            if target_gpos[position_idx] <= glimit[1][position_idx]:
                                target_gpos[position_idx] += POSITION_INSERMENT * direction

                    elif k in key_to_joint_decrease:
                        position_idx = key_to_joint_decrease[k]
                        if position_idx == 1 or position_idx == 5:
                            position_idx = 0 if position_idx == 1 else 5
                            if target_qpos[position_idx] > qlimit[0][position_idx] - JOINT_INCREMENT * direction:
                                target_qpos[position_idx] += JOINT_INCREMENT * direction
                        elif position_idx == 4 or position_idx == 3:
                            if target_gpos[position_idx] <= glimit[1][position_idx]:
                                target_gpos[position_idx] += POSITION_INSERMENT * direction * 4
                        else:
                            if target_gpos[position_idx] >= glimit[0][position_idx]:
                                target_gpos[position_idx] += POSITION_INSERMENT * direction

            print("target_gpos:", [f"{x:.3f}" for x in target_gpos])
            fd_qpos = np.concatenate(([0.0,], mjdata.qpos[qpos_indices][1:5]))
            qpos_inv = lerobot_IK(fd_qpos, target_gpos)

            if np.all(qpos_inv != -1.0):  # Check if IK solution is valid
                target_qpos = np.concatenate((target_qpos[0:1], qpos_inv[1:5], target_qpos[5:]))
                mjdata.qpos[qpos_indices] = target_qpos

                mujoco.mj_step(mjmodel, mjdata)
                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(mjdata.time % 2)
                viewer.sync()

                joint_angles = np.rad2deg(target_qpos)
                joint_angles[1] = -joint_angles[1]
                joint_angles[0] = -joint_angles[0]
                joint_angles[4] = -joint_angles[4]
                follower_arm.write("Goal_Position", joint_angles)
                position = follower_arm.read("Present_Position")

                target_gpos_last = target_gpos.copy()
            else:
                target_gpos = target_gpos_last.copy()

            print()
            time_until_next_step = mjmodel.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

except KeyboardInterrupt:
    print("User interrupted the simulation.")
finally:
    listener.stop()
    follower_arm.disconnect()