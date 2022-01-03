# Niryo One utility Functions
import numpy as np
from numpy.linalg import inv
import math

'''
Forward Kinematics for NiryoOne
Input: Array with 6DOF joint angles
Output: 6 Homogeneous Transformation Matrices
Each matrix gives the position and orientation
of the 'end_effector' relative to de 'base_link'
'''
def ForwardKinematics(thetas):
	# Separate the Theta Values
	t1 = thetas[0]
	t2 = thetas[1]
	t3 = thetas[2]
	t4 = thetas[3]
	t5 = thetas[4]
	t6 = thetas[5]
	# Rotation around Z axis
	tf_01 = np.array([[ math.cos(t1), -math.sin(t1),    0, 0.000],
					  [ math.sin(t1),  math.cos(t1),    0, 0.000],
					  [            0,             0,    1, 0.103],
					  [            0,             0,    0,     1]])
	# Rotation around Y axis
	tf_12 = np.array([[ math.cos(t2),   0, -math.sin(t2),  0.000],
					  [            0,   1,             0,  0.000],
					  [ math.sin(t2),   0,  math.cos(t2),  0.080],
					  [            0,   0,             0,      1]])
	# Rotation around Y axis
	tf_23 = np.array([[ math.cos(t3),   0, -math.sin(t3),  0.000],
					  [            0,   1,             0,  0.000],
					  [ math.sin(t3),   0,  math.cos(t3),  0.210],
					  [            0,   0,             0,      1]])
	# Rotation around X axis
	tf_34 = np.array([[   1,             0,             0, 0.0415],
					  [   0,  math.cos(t4), -math.sin(t4), 0.0000],
					  [   0,  math.sin(t4),  math.cos(t4), 0.0300],
					  [   0,             0,             0,      1]])
	# Rotation around Y axis
	tf_45 = np.array([[  math.cos(t5),    0, -math.sin(t5), 0.180],
					  [             0,    1,            0,  0.000],
					  [  math.sin(t5),    0, math.cos(t5),  0.000],
					  [             0,    0,            0,     1]])
	# Rotation around X axis
	tf_56 = np.array([[   1,             0,             0,  0.0237],
					  [   0,  math.cos(t6), -math.sin(t6),  0.0000],
					  [   0,  math.sin(t6),  math.cos(t6), -0.0055],
					  [   0,             0,             0,       1]])
	# Base to End TF
	tf_46 = np.matmul(tf_45, tf_56)
	tf_36 = np.matmul(tf_34, tf_46)
	tf_26 = np.matmul(tf_23, tf_36)
	tf_16 = np.matmul(tf_12, tf_26)
	tf_06 = np.matmul(tf_01, tf_16)
	# Other Auxiliar TF's 
	tf_02 = np.matmul(tf_01, tf_12)
	tf_13 = np.matmul(tf_12, tf_23)
	tf_03 = np.matmul(tf_01, tf_13)
	tf_24 = np.matmul(tf_23, tf_34)
	tf_14 = np.matmul(tf_12, tf_24)
	tf_04 = np.matmul(tf_01, tf_14) 
	tf_35 = np.matmul(tf_34, tf_45)
	tf_25 = np.matmul(tf_23, tf_35)
	tf_15 = np.matmul(tf_12, tf_25)
	tf_05 = np.matmul(tf_01, tf_15)

	return tf_01, tf_02, tf_03, tf_04, tf_05, tf_06


'''
Simplified Inverse Kinematics for NiryoOne
Input: Cartesion Pose (X,Y,Z)
Output: Angles of the first 3 joints
'''
def InverseKinematics(X, Y, Z):

	t1 = atan2(Y,X)

	return t1
