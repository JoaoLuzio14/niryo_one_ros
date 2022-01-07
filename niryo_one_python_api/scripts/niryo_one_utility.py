# Niryo One Utility Functions
import numpy as np
from numpy.linalg import inv
import math

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

	return theta1, theta2, theta3, theta4, theta5, theta6
