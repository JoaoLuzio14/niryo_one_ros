#!/usr/bin/env python

'''

- Robotics Course - Instituto Superior Tecnico
- Lab 1 (P2 - 2021/2022)
- 93096 - Joao Miguel Barradas Luzio
- 93125 - Marcelo Jose da Silva Braco Forte
- 93771 - Filipe De Jesus Pereira Ferraz

'''

from niryo_one_python_api.niryo_one_api import *
import rospy
import math
import time
    
rospy.init_node('niryo_one_example_python_api')  
niryo = NiryoOne()
    
try:
    # Robot Calibration 
    niryo.calibrate_auto()
    time.sleep(1)

    # Start at 'home' position (to define!!!)
    niryo.set_arm_max_velocity(20) # Define an apropriate velocity for the task!!!
    home_position = [0.0, 0.0, -math.pi/4.0, 0.0, math.radians(45), 0.0]
    niryo.move_joints(home_position)


    # Place above blocks origin location (to continue...)



except NiryoOneException as e:
    print e
