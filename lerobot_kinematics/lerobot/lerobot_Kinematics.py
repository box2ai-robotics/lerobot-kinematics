# code by LinCC111 Boxjod 2025.1.13 Box2AI-Robotics copyright 盒桥智能 版权所有

import numpy as np
import math
from math import sqrt as sqrt
from spatialmath import SE3, SO3
from lerobot_kinematics.ET import ET
from scipy.spatial.transform import Rotation as R

# Retain 15 decimal places and round off after the 15th place
def atan2(first, second):
    return round(math.atan2(first, second), 3)

def sin(radians_angle):
    return round(math.sin(radians_angle), 3)

def cos(radians_angle):
    return round(math.cos(radians_angle), 3)

def acos(value):
    return round(math.acos(value), 3)

def round_value(value):
    return round(value, 3)

def create_so100():
    # to joint 1
    # E1 = ET.tx(0.0612)
    # E2 = ET.tz(0.0598)
    # E3 = ET.Rz()
    
    # to joint 2
    E4 = ET.tx(0.02943)
    E5 = ET.tz(0.05504)
    E6 = ET.Ry()
    
    # to joint 3
    E7 = ET.tx(0.1127)
    E8 = ET.tz(-0.02798)
    E9 = ET.Ry()

    # to joint 4
    E10 = ET.tx(0.13504)
    E11 = ET.tz(0.00519)
    E12 = ET.Ry()
    
    # to joint 5
    E13 = ET.tx(0.0593)
    E14 = ET.tz(0.00996)
    E15 = ET.Rx()  
    
    # E17 = ET.tx(0.09538)
    # to gripper
    
    so100 = E4 * E5 * E6 * E7 * E8 * E9 * E10 * E11 * E12 * E13 * E14 * E15 # E1 * E2 * E3 * E17 
    
    # Set joint limits
    so100.qlim = [[-3.14158, -0.2,     -1.5, -3.14158], 
                  [ 0.2,      3.14158,  1.5,  3.14158]]
    
    return so100

def create_so101():
    # to joint 1
    # E1 = ET.tx(0.0612)
    # E2 = ET.tz(0.0598)
    # E3 = ET.Rz()
    
    # to joint 2
    E4 = ET.tx(0.02943)
    E5 = ET.tz(0.05504)
    E6 = ET.Ry()
    
    # to joint 3
    E7 = ET.tz(0.1127)
    E8 = ET.tx(0.02798)
    E9 = ET.Ry()

    # to joint 4
    E10 = ET.tx(0.13504)
    E11 = ET.tz(0.00519)
    E12 = ET.Ry()
    
    # to joint 5
    E13 = ET.tx(0.0593)
    E14 = ET.tz(0.00996)
    E15 = ET.Rx()  
    
    # E17 = ET.tx(0.09538)
    # to gripper
    
    so100 = E4 * E5 * E6 * E7 * E8 * E9 * E10 * E11 * E12 * E13 * E14 * E15 # E1 * E2 * E3 * E17 
    
    # Set joint limits
    so100.qlim = [[-1.57, -1.57, -1.5, -3.14158], 
                  [ 1.57,  1.57,  1.5,  3.14158]]
    
    return so100

def get_robot(robot="so100"):
    
    if robot == "so100":
        return create_so100()
    elif robot == "so101":
        return create_so101()
    else:
        print(f"Sorry, we don't support {robot} robot now")
        return None

def lerobot_FK(qpos_data, robot):
    if len(qpos_data) != len(robot.qlim[0]):
        raise Exception("The dimensions of qpose_data are not the same as the robot joint dimensions")
    # Get the end effector's homogeneous transformation matrix (T is an SE3 object)
    T = robot.fkine(qpos_data)
    
    # Extract position (X, Y, Z) — use SE3 object's attribute
    X, Y, Z = T.t  # Directly use t attribute to get position (X, Y, Z)
    
    # Extract rotation matrix (T.A) and calculate Euler angles (alpha, beta, gamma)
    R = T.R  # Get the rotation part (3x3 matrix)

    # Calculate Euler angles
    beta = atan2(-R[2, 0], sqrt(R[0, 0]**2 + R[1, 0]**2))
    
    if cos(beta) != 0:  # Ensure no division by zero
        alpha = atan2(R[1, 0] / cos(beta), R[0, 0] / cos(beta))
        gamma = atan2(R[2, 1] / cos(beta), R[2, 2] / cos(beta))
    else:  # When cos(beta) is zero, singularity occurs
        alpha = 0
        gamma = atan2(R[0, 1], R[1, 1])
    
    return np.array([X, Y, Z, gamma, beta, alpha])
    
def lerobot_IK(q_now, target_pose, robot):
    if len(q_now) != len(robot.qlim[0]):
        raise Exception("The dimensions of qpose_data are not the same as the robot joint dimensions")
    # R = SE3.RPY(target_pose[3:])
    # T = SE3(target_pose[:3]) * R
    
    x, y, z, roll, pitch, yaw = target_pose
    r = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)  # 欧拉角的顺序是 XYZ
    R_mat = r.as_matrix()  # 获取旋转矩阵
    T = np.eye(4)
    T[:3, :3] = R_mat
    T[:3, 3] = [x, y, z]
    
    sol = robot.ikine_LM(
            Tep=T, 
            q0=q_now,
            ilimit=10,  # 10 iterations
            slimit=2,  # 1 is the limit
            tol=1e-3)  # tolerance for convergence
    
    if sol.success:
        # If IK solution is successful, 
        q = sol.q
        q = smooth_joint_motion(q_now, q, robot)
        # print(f'{q=}')
        return q, True
    else:
        # If the target position is unreachable, IK fails
        print(f'IK fails')
        return -1 * np.ones(len(q_now)), False
    
def smooth_joint_motion(q_now, q_new, robot):
    q_current = q_now
    max_joint_change = 0.1 
    
    for i in range(len(q_new)):
        delta = q_new[i] - q_current[i]
        if abs(delta) > max_joint_change:
            delta = np.sign(delta) * max_joint_change
        q_new[i] = q_current[i] + delta
    
    robot.q = q_new
    return q_new
