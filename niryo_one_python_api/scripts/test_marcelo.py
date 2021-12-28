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


default_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  

home_position = [0.0, 0.0, -math.pi/4.0, 0.0, math.radians(45), 0.0]

origin_up = 	[-1.03, -0.42, -0.67, 0, -0.465, -1]



matrix_origin = np.matrix([  [-1.03, -0.58, -0.66, 0, -0.31, -1],              #origin_position_1
                            [-1.03, -0.74, -0.63, 0, -0.175, -1],              #origin_position_2
                            [-1.03, -0.905, -0.575, 0, -0.075, -1],                  #origin_position_3
                            [-1.03, -1.07, -0.499, 0, 0.02, -1]    ] )           #origin_position_4


matrix_target_up = np.matrix([   [0.238, -0.450, -0.625, 0, -0.5, 0.23],             #taget_up_1
                                [0.206, -0.554, -0.442, 0, -0.58, 0.206],             #taget_up_2
                                [0.383, -0.484, -0.568, 0, -0.523, 0.370],             #taget_up_3
                                [0.334, -0.584, -0.389, 0, -0.61, 0.33]   ] )           #taget_up_4     

matrix_target_position = np.matrix([ [0.238, -1.074, -0.460, 0, -0.042, 0.23],          #taget_position_1
                                    [0.206, -1.113, -0.285, 0, -0.183, 0.206],          #taget_position_2
                                    [0.383, -1.084, -0.408, 0, -0.083, 0.37],           #taget_position_3
                                    [0.334, -1.127, -0.234, 0, -0.220, 0.33]  ] )      #target_position_4
                    


origin = matrix_origin.tolist()
target_up = matrix_target_up.tolist()
target_position = matrix_target_position.tolist()



n.set_arm_max_velocity(40) # Define an apropriate velocity for the task!!!

n.move_joints(home_position)


try:
    for i in range(4):
       
        n.move_joints(origin_up)
        n.set_arm_max_velocity(10)
        n.move_joints(origin[i])
        time.sleep(1)
        n.move_joints(origin_up)

        n.move_joints(target_up[i])
        n.move_joints(target_position[i])
        time.sleep(1)
        n.set_arm_max_velocity(40)
        n.move_joints(target_up[i])
        
        #//////////////////////////
        
    
    time.sleep(1)

    n.move_joints(home_position)

except  NiryoOneException  as e:
    print(e)

print ('\n \nDone. Thank you!')
