#!/usr/bin/env python

'''

- Robotics Course - Instituto Superior Tecnico
- Lab 1 (P2 - 2021/2022)
- 93096 - Joao Miguel Barradas Luzio
- 93125 - Marcelo Jose da Silva Braco Forte
- 93771 - Filipe De Jesus Pereira Ferraz

'''

import  rospy
import  math
import  time
import numpy as np 
from niryo_one_python_api.niryo_one_api import *

rospy.init_node('niryo_one_example_python_api')

print("\n \n--- Start")

n = NiryoOne ()
n.calibrate_auto ()
time.sleep(1)
print ('\n \nCalibration finished !\n')

n.activate_learning_mode(False)
print ('\n \nLearning mode activated?')
print (n.get_learning_mode())

default_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  
home_position = [0.0, 0.0, -math.pi/4.0, 0.0, math.radians(45), 0.0]
origin_up = [-1.031, -0.4, -0.669, -0.027, -0.491, -1.002]

joint_limits = np.matrix([  [   -175*math.pi/180,   175*math.pi/180],   
                            [    -90*math.pi/180,  36.7*math.pi/180],
                            [    -80*math.pi/180,    90*math.pi/180],
                            [   -175*math.pi/180,   175*math.pi/180],
                            [   -100*math.pi/180,   110*math.pi/180],
                            [ -147.5*math.pi/180, 147.5*math.pi/180]])

matrix_origin = np.matrix([ [-1.031, -0.576, -0.670, -0.027, -0.354, -1.073],              #origin_position_1
                            [-1.031, -0.728, -0.673,  0.524, -0.111, -1.518],              #origin_position_2
                            [-1.031, -0.900, -0.610,  0.638, -0.020, -1.665],              #origin_position_3
                            [-1.031, -1.066, -0.531,  0.381,  0.015,  1.620]])             #origin_position_4


matrix_target_up = np.matrix([  [  0.178, -0.927, -0.323,  0.003, -0.294, -1.326],             #target_up_1
                                [  0.243, -0.805, -0.523,  0.408, -0.278, -1.645],             #target_up_2
                                [ -0.012, -0.835, -0.403,  0.176, -0.385,  1.473],             #target_up_3
                                [  0.270, -0.771, -0.571, -0.575, -0.243,  2.257]])            #target_up_4     

matrix_target_position = np.matrix([ [  0.177, -1.138, -0.251,  0.003, -0.147, -1.367],          #target_position_1
                                     [  0.243, -1.094, -0.426,  0.408, -0.051, -1.665],          #target_position_2
                                     [  0.008, -1.129, -0.286,  0.175, -0.111,  1.473],          #target_position_3
                                     [  0.280, -1.089, -0.451, -0.497, -0.101,  2.136]])         #target_position_4 #MALLLLLLLLLLL
                    


origin = matrix_origin.tolist()
target_up = matrix_target_up.tolist()
target_position = matrix_target_position.tolist()

n.set_arm_max_velocity(20)  #defines the velocity of the arm
n.move_joints(home_position)

try:
    for i in range(4): # Execute the task
       
        n.move_joints(origin_up)
        n.move_joints(origin[i])
        time.sleep(1)
        n.move_joints(origin_up)
        n.move_joints(target_up[i])
        n.move_joints(target_position[i])
        time.sleep(1)
        n.move_joints(target_up[i])

        #//////////////////////////
        
    # Final Dance
    n.move_joints(home_position)
    joints = [0.0, 0.0, -math.pi/4.0, 0.0, -0.5, 0.0]
    n.move_joints(joints)
    joints = [0.0, 0.0, -math.pi/4.0, 0.0, 1.2, 0.0]
    n.move_joints(joints) 
    n.move_joints(home_position)
    
except  NiryoOneException  as e:
    print(e)

print ('\n \nDone. Thank you!')