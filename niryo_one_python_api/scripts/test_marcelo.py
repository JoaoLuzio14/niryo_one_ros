#!/usr/bin/env  python

'''
- Robotics Course - Instituto Superior Tecnico
- Lab 1 (P2 - 2021/2022)
- 93096 - Joao Miguel Barradas Luzio
- 93125 - Marcelo Jose da Silva Braco Forte
- 93771 - Filipe De Jesus Pereira Ferraz
'''

from niryo_one_python_api.niryo_one_api import *
import  rospy
import  math
import  time
import numpy as np 

# Generates errors for each joint given a Gaussian distribution
def joint_errors(mean, deviation):

    errors = np.random.normal(mean, deviation, 6)
    return errors

'''
This script will grab the 4 pilled blocks
and move them to another chosen location.
'''

# Define error parameters (just random error, ignoring joint bias)
mean_val = 0.0 # Centered Value
stand_dev = 0.002 # 0.001 < SD < 0.005 for realistic variations
#P(-0.004 < X < 0.004) = 0.95

# Define the joint angles for the desired trajectory

default_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  
home_position = [0.0, 0.0, -math.pi/4.0, 0.0, math.radians(45), 0.0]
start_up = [-1.03, -0.42, -0.67, 0, -0.465, -1]

matrix_start = np.matrix([ [ -1.030, -0.580, -0.660, 0, -0.310, -1],            #start_position_1
                            [ -1.030, -0.740, -0.630, 0, -0.175, -1],            #start_position_2
                            [ -1.030, -0.905, -0.575, 0, -0.075, -1],            #start_position_3
                            [ -1.030, -1.070, -0.499, 0,  0.020, -1]])           #start_position_4

matrix_target_up = np.matrix([  [ 0.238, -0.450, -0.625, 0, -0.500, 0.230],             #taget_up_1
                                [ 0.206, -0.554, -0.442, 0, -0.580, 0.206],             #taget_up_2
                                [ 0.383, -0.484, -0.568, 0, -0.523, 0.370],             #taget_up_3
                                [ 0.334, -0.584, -0.389, 0, -0.610, 0.330]])            #taget_up_4 

matrix_target_position = np.matrix([[ 0.238, -1.074, -0.460, 0, -0.042, 0.230],       #target_position_1
                                    [ 0.206, -1.113, -0.285, 0, -0.183, 0.206],       #target_position_2
                                    [ 0.383, -1.084, -0.408, 0, -0.083, 0.370],       #target_position_3
                                    [ 0.334, -1.127, -0.234, 0, -0.220, 0.330]])      #target_position_4
                    
start = matrix_start.tolist()
target_up = matrix_target_up.tolist()
target_position = matrix_target_position.tolist()

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
            niryo.set_arm_max_velocity(10)
            print ('\n \nGrabbing block number {}...\n'.format(i+1))
            niryo.move_joints(start[i] + joint_errors(mean_val, stand_dev))
            time.sleep(1)
            niryo.move_joints(start_up + joint_errors(mean_val, stand_dev))
            print ('Moving block to the target area {}...\n'.format(i+1))
            niryo.set_arm_max_velocity(10)
            niryo.move_joints(target_up[i] + joint_errors(mean_val, stand_dev))
            niryo.set_arm_max_velocity(10)
            niryo.move_joints(target_position[i] + joint_errors(mean_val, stand_dev))
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

print ("\n \n--- End ---\n \nThank you!")
