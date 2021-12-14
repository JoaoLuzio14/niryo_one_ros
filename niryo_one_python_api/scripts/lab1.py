#!/usr/bin/env python

'''

- Robotics Course - Instituto Superior Tecnico
- Lab 1 (P2 - 2021/2022)
- 93096 - Joao Miguel Barradas Luzio
- 93125 - Marcelo Jose da Silva Braco Forte
- 93771 - Filipe De Jesus Pereira Ferraz

'''

from niryo_one_python_api.niryo_one_api import *
import tf.transformations as tform
import numpy as np
import rospy
import math
import time
    
rospy.init_node('niryo_one_example_python_api')  
niryo = NiryoOne()

# Define Positions
home_position = [0.0, 0.0, -math.pi/4.0, 0.0, math.radians(45), 0.0]
default_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
origin_position = [-1.038, -1.086, -0.496, 0.128, -0.010, 0.349]
origin_up_pose = [-1.038, -0.539, -0.604, 0.128, -0.450, 0.364]
target_up_pose = [0.296, -0.637, -0.373, 0.078, -0.526, 0.162]
target_position = [0.296, -1.155, -0.214, 0.089, -0.187, 0.162]

    
try:
    # Robot Calibration 
    niryo.calibrate_auto()
    time.sleep(1)
    niryo.set_arm_max_velocity(20) # Define an apropriate velocity for the task

    # Start at 'home' position
    niryo.move_joints(home_position)
    time.sleep(2)

    # Place above blocks origin location
    niryo.move_joints(origin_position)
    time.sleep(2)

    # Lift Up
    niryo.move_joints(origin_up_pose)
    time.sleep(2)

    # Move to position above target
    niryo.move_joints(target_up_pose)
    time.sleep(2)

    # Down the object to the target
    niryo.move_joints(target_position)
    time.sleep(2)

    # End in default position
    niryo.move_joints(default_position)


except NiryoOneException as e:
    print e
