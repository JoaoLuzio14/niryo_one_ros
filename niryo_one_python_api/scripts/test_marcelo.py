#!/usr/bin/env  python
import  rospy
import  math
import  time


from niryo_one_python_api.niryo_one_api import *

rospy.init_node('niryo_one_example_python_api')

print("--- Start")

n = NiryoOne ()
n.calibrate_auto ()
time.sleep(1)
print ("Calibration finished !\n")
c = 0

n.set_arm_max_velocity(20) # Define an apropriate velocity for the task!!!
home_position = [0.0, 0.0, -math.pi/4.0, 0.0, math.radians(45), 0.0]

try:
    for i in range(1):
        print('-test  counting'.format(c))
        c = c + 1
        joints = n.get_joints ()
        print(joints)

        n.move_joints(home_position)

        joints = [-1.02, 0, -math.pi/4.0, 0, 0, 0]
        n.move_joints(joints)
        
        joints = [-1.02, -1.145, -0.485, -1.096, 0, -0.005]
        n.move_joints(joints)
    
        time.sleep(1)

        joints = [-1.02, -0.5, -1, -1.096, 0, -0.005]
        n.move_joints(joints)

        joints = [0.268, -0.5, -1, -1.096, 0, -0.005]
        n.move_joints(joints)

        joints = [0.268, -1.145, -0.333, -0.067, -0.1, 0.223]
        n.move_joints(joints)

        time.sleep(1)



    n.move_joints(home_position)
    joints = [0.0, 0.0, -math.pi/4.0, 0.0, -0.5, 0.0]
    n.move_joints(joints)
    joints = [0.0, 0.0, -math.pi/4.0, 0.0, 1.2, 0.0]
    n.move_joints(joints)
    n.move_joints(home_position)
    
except  NiryoOneException  as e:
    print(e)