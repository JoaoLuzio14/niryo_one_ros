#!/usr/bin/env  python
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

#CODIGO PARA EXPERIMENTAR!!
default_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  

home_position = [0.0, 0.0, -math.pi/4.0, 0.0, math.radians(45), 0.0]

matrix_origin_up = np.matrix ([	[-1.03, -0.42, -0.67, 0, -0.465, -1],
                                [-1.03, -0.58, -0.66, 0, -0.31, -1],
                                [-1.03, -0.74, -0.63, 0, -0.175, -1],
                                [-1.03, -0.905, -0.575, 0, -0.075, -1] ])




matrix_origin = np.matrix([  [-1.03, -0.58, -0.66, 0, -0.31, -1],              #origin_position_1
                            [-1.03, -0.74, -0.63, 0, -0.175, -1],              #origin_position_2
                            [-1.03, -0.905, -0.575, 0, -0.075, -1],                  #origin_position_3
                            [-1.03, -1.07, -0.499, 0, 0.02, -1]    ] )           #origin_position_4


matrix_target_up = np.matrix([   [0.178, -0.927, -0.323, 0.003, -0.294, -1.326],             #taget_up_1
                                [0.243, -0.805, -0.523, 0.408, -0.278, -1.645],             #taget_up_2
                                [-0.012, -0.835, -0.403, 0.176, -0.385, 1.473],             #taget_up_3
                                [0.28, -0.771, -0.571, -0.575, -0.243, 2.257]   ] )           #taget_up_4     

matrix_target_position = np.matrix([ [0.178, -1.138, -0.251, 0.003, -0.147, -1.367],          #taget_position_1
                                    [0.243, -1.094, -0.426, 0.408, -0.051, -1.665],          #taget_position_2
                                    [0.008, -1.129, -0.286, 0.175, -0.111, 1.473],           #taget_position_3
                                    [0.28, -1.089, -0.451, -0.497, -0.101, 2.136]  ] )      #target_position_4 #MALLLLLLLLLLL
                    


origin = matrix_origin.tolist()
target_up = matrix_target_up.tolist()
target_position = matrix_target_position.tolist()
origin_up= matrix_origin_up.tolist()

c=1
n.set_arm_max_velocity(80) # Define an apropriate velocity for the task!!!
i=0
n.move_joints(home_position)


try:
    for i in range(4):
       
        n.move_joints(origin_up[i])
        n.move_joints(origin[i])
        time.sleep(1)
        n.move_joints(origin_up[i])

        n.move_joints(home_position)

        n.move_joints(target_up[c])
        n.move_joints(target_position[c])
        time.sleep(1)
        n.move_joints(target_up[c])
        c=c+1
        #//////////////////////////
        
    #dancinha do fim, tirar se nao quiserem colocar
    # n.move_joints(home_position)
    # joints = [0.0, 0.0, -math.pi/4.0, 0.0, -0.5, 0.0]
    # n.move_joints(joints)
    # joints = [0.0, 0.0, -math.pi/4.0, 0.0, 1.2, 0.0]
    # n.move_joints(joints)
    # n.move_joints(home_position)
    
    time.sleep(1)

except  NiryoOneException  as e:
    print(e)

print ('\n \nDone. Thank you!')

# to comment a portion of a code, select that portion and do ctrl + /
