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
origin_position = [-1.020, -1.107, -0.485, -1.096, 0.0, -0.005]
target_position = [0.268, -1.145, -0.333, -0.067, -0.020, 0.223]
    
try:
    # Robot Calibration 
    niryo.calibrate_auto()
    time.sleep(1)

    # Start at 'home' position
    niryo.set_arm_max_velocity(20) # Define an apropriate velocity for the task
    niryo.move_joints(home_position)
    time.sleep(2)

    # Place above blocks origin location
    niryo.move_joints(origin_position)
    time.sleep(2)

    # Drag object directly to the target location
    niryo.move_joints(target_position)
    time.sleep(2)

    niryo.move_joints(default_position)


except NiryoOneException as e:
    print e
