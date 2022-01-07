#!/usr/bin/env  python

'''
- Robotics Course - Instituto Superior Tecnico
- Lab 1 (P2 - 2021/2022)
- 93096 - Joao Miguel Barradas Luzio
- 93125 - Marcelo Jose da Silva Braco Forte
- 93771 - Filipe De Jesus Pereira Ferraz
'''

# Usage inside built catkin workspace: rosrun niryo_one_python_api lab.py
# For usage simplicity, this script takes no input arguments

from niryo_one_python_api.niryo_one_api import *
import  rospy
import  math
import  time
import numpy as np

#---------------------------------------------------------------------------------------------------------------------------------------------------
#---------------------------------------------------------------------------------------------------------------------------------------------------

'''
Forward Kinematics for NiryoOne
Input: 6DOF joint angles in radians
Output: Position of the End Effector
Each matrix gives the position and orientation
of the 'end_effector' relative to de 'base_link'
'''
def ForwardKinematics(t):

    if len(t) != 6:
        print("\nInvalid Dimensions for Joint Array!\n")
        return [0.0, 0.0, 0.0]

    # Rotation around Z axis
    tf_01 = np.array([[ math.cos(t[0]), -math.sin(t[0]),    0, 0.000],
                      [ math.sin(t[0]),  math.cos(t[0]),    0, 0.000],
                      [              0,               0,    1, 0.103],
                      [              0,               0,    0,     1]])

    # Rotation around Y axis
    tf_12 = np.array([[ math.cos(t[1]),   0, -math.sin(t[1]),  0.000],
                      [              0,   1,               0,  0.000],
                      [ math.sin(t[1]),   0,  math.cos(t[1]),  0.080],
                      [              0,   0,               0,      1]])

    # Rotation around Y axis
    tf_23 = np.array([[ math.cos(t[2]),   0, -math.sin(t[2]),  0.000],
                      [              0,   1,               0,  0.000],
                      [ math.sin(t[2]),   0,  math.cos(t[2]),  0.210],
                      [              0,   0,               0,      1]])

    # Rotation around X axis
    tf_34 = np.array([[   1,               0,               0, 0.0415],
                      [   0,  math.cos(t[3]), -math.sin(t[3]), 0.0000],
                      [   0,  math.sin(t[3]),  math.cos(t[3]), 0.0300],
                      [   0,               0,               0,      1]])

    # Rotation around Y axis
    tf_45 = np.array([[  math.cos(t[4]),    0, -math.sin(t[4]),  0.180],
                      [               0,    1,               0,  0.000],
                      [  math.sin(t[4]),    0,  math.cos(t[4]),  0.000],
                      [               0,    0,               0,     1]])

    # Rotation around X axis
    tf_56 = np.array([[   1,               0,               0,  0.0237],
                      [   0,  math.cos(t[5]), -math.sin(t[5]),  0.0000],
                      [   0,  math.sin(t[5]),  math.cos(t[5]), -0.0055],
                      [   0,               0,               0,       1]])

    # Base to End TF
    tf_46 = np.matmul(tf_45, tf_56)
    tf_36 = np.matmul(tf_34, tf_46)
    tf_26 = np.matmul(tf_23, tf_36)
    tf_16 = np.matmul(tf_12, tf_26)
    tf_06 = np.matmul(tf_01, tf_16)

    position = np.dot(tf_06, np.transpose(np.array([0, 0, 0, 1])))
    position = position[0:3]
    for i in range(len(position)):
        position[i] = round(position[i], 4)

    return position 

#---------------------------------------------------------------------------------------------------------------------------------------------------

'''
Simplified Inverse Kinematics for NiryoOne
Input: Cartesion Pose (X,Y,Z)
Output: Angles of the 6 joints
'''
def InverseKinematics(X, Y, Z):

    # Calculate Dimensions
    a1 = 0.183
    d = Z - a1 + 0.0237
    a2 = 0.210
    L = 0.2215
    K = 0.030
    r = math.sqrt(np.power(X, 2) + np.power(Y, 2)) + 0.0055
    R = math.sqrt(np.power(r, 2) + np.power(d, 2))
    beta = math.atan2(K,L)
    a3 = math.sqrt(np.power(L,2)+np.power(K,2))

    # Convenient Solution for the Task
    theta3 = np.arcsin((np.power(R, 2) - np.power(a2, 2) - np.power(a3, 2))/(2*a2*a3)) - beta
    theta2 = math.atan2(d,r) + np.arccos((np.power(R, 2) + np.power(a2, 2) - np.power(a3, 2))/(2*a2*R)) - math.pi/2
    theta5 = np.arccos((np.power(R, 2) + np.power(a3, 2) - np.power(a2, 2))/(2*a3*R)) - math.pi/2 - math.atan2(d,r) + beta

    # Fixed Values
    theta1 = math.atan2(Y, X) # -90 < t1 < +90 degree
    theta6 = theta1 # Duo to Simulator Referencial Convencion
    theta4 = 0.0 # Fixed

    thetas = [theta1, theta2, theta3, theta4, theta5, theta6]
    return thetas

#---------------------------------------------------------------------------------------------------------------------------------------------------

'''
Generates errors for each joint given a Gaussian distribution
Input: Mean Value and Standard Deviation
Output: Array with 6 random error values (one for each joint)
'''
def joint_errors(mean, deviation):

    errors = np.random.normal(mean, deviation, 6)
    return errors

#---------------------------------------------------------------------------------------------------------------------------------------------------

'''
Land the object in a pseudo-vertical trajectory
Input: step, speed, and start and finish positions
'''
def land(niryo, step, speed, begin, end):

    pose = begin
    pose[2] -= step
    niryo.set_arm_max_velocity(speed)

    while pose[2] > end[2]:
        thetas = InverseKinematics(pose[0], pose[1], pose[2])
        niryo.move_joints(thetas + joint_errors(0.0, 0.001))
        time.sleep(0.01)
        pose[2] -= step
    
#---------------------------------------------------------------------------------------------------------------------------------------------------
#---------------------------------------------------------------------------------------------------------------------------------------------------

'''
This script will grab the 4 pilled blocks
and move them to another chosen location.
'''
if __name__ == "__main__":

    # Define error parameters (just random error, ignoring joint bias)
    mean_val = 0.0 # Centered Value
    stand_dev = 0.005 # 0.001 < SD < 0.005 for realistic variations
    # P(-0.004 < X < 0.004) = 0.95

    # Define the joint angles for the desired trajectory

    default_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  
    home_position = [0.0, 0.0, -math.pi/4.0, 0.0, math.radians(45), 0.0]
    start_up = [-1.030, -0.420, -0.670, 0, -0.465, -1]

    matrix_start = np.matrix([  [ -1.030, -0.580, -0.660, 0, -0.310, -1],        
                                [ -1.030, -0.740, -0.630, 0, -0.175, -1],     
                                [ -1.030, -0.905, -0.575, 0, -0.075, -1], 
                                [ -1.030, -1.070, -0.499, 0,  0.020, -1]])           

    matrix_target_up = np.matrix([  [ 0.238, -0.450, -0.625, 0, -0.500, 0.230],     
                                    [ 0.206, -0.554, -0.442, 0, -0.580, 0.206],        
                                    [ 0.383, -0.484, -0.568, 0, -0.523, 0.370],
                                    [ 0.334, -0.584, -0.389, 0, -0.610, 0.330]]) 

    matrix_target_position = np.matrix([[ 0.238, -1.074, -0.460, 0, -0.042, 0.230],
                                        [ 0.206, -1.113, -0.285, 0, -0.183, 0.206],
                                        [ 0.383, -1.084, -0.408, 0, -0.083, 0.370],
                                        [ 0.334, -1.127, -0.234, 0, -0.220, 0.330]])

    up_pose = np.matrix([[ 0.211,  0.051,  0.167, 0, math.radians(90), 0],
                         [ 0.245,  0.051,  0.167, 0, math.radians(90), 0],
                         [ 0.211,  0.085,  0.167, 0, math.radians(90), 0],
                         [ 0.245,  0.085,  0.167, 0, math.radians(90), 0]])

    down_pose = np.matrix([[ 0.211,  0.051,  0.039, 0, math.radians(90), 0],
                           [ 0.245,  0.051,  0.039, 0, math.radians(90), 0],
                           [ 0.211,  0.085,  0.039, 0, math.radians(90), 0],
                           [ 0.245,  0.085,  0.039, 0, math.radians(90), 0]])
                        
    start = matrix_start.tolist()
    target_up = matrix_target_up.tolist()
    target_position = matrix_target_position.tolist()
    begin = up_pose.tolist()
    end = down_pose.tolist()

    # Start executing the planned motion ... 

    print("\n \n--- Start ---")

    rospy.init_node('niryo_one_example_python_api')
    niryo = NiryoOne()

    print ('\n \nCalibrating Niryo...\n')
    niryo.calibrate_auto()
    time.sleep(1)
    print ('\n \nCalibration finished !\n')
    niryo.activate_learning_mode(False)

    if niryo.get_learning_mode() == False:

        try:
            print ('\n \nStarting the action...\n')
            niryo.set_arm_max_velocity(40)
            niryo.move_joints(home_position)

            for i in range(4):
               
                niryo.move_joints(start_up + joint_errors(mean_val, stand_dev))
                niryo.set_arm_max_velocity(3)

                print ('\n \nGrabbing block number {}...\n'.format(i+1))
                niryo.move_joints(start[i] + joint_errors(mean_val, stand_dev))
                pose = ForwardKinematics(start[i])
                print ('Position: X:{} Y:{} Z:{} \n'.format(pose[0], pose[1], pose[2]))
                time.sleep(1)
                niryo.move_joints(start_up + joint_errors(mean_val, stand_dev))

                print ('Moving block to the target area {}...\n'.format(i+1))
                niryo.set_arm_max_velocity(10)
                niryo.move_joints(target_up[i] + joint_errors(mean_val, stand_dev))
                land(niryo, 0.005, 5, begin[i], end[i])
                niryo.move_joints(target_position[i] + joint_errors(mean_val, stand_dev))
                pose = ForwardKinematics(target_position[i])
                print ('Position: X:{} Y:{} Z:{} \n'.format(pose[0], pose[1], pose[2]))

                print ('Please unattach block {} from the gripper!\n'.format(i+1))
                time.sleep(3)

                niryo.set_arm_max_velocity(40)
                niryo.move_joints(target_up[i] + joint_errors(mean_val, stand_dev))
                print ('Successfully dropped block number {}!\n'.format(i+1))
            
            time.sleep(1)
            niryo.move_joints(home_position)
            print ('\n \nSuccess!\n')

        except  NiryoOneException  as e:
            print(e)

    print ("\n \n--- End ---\n")
