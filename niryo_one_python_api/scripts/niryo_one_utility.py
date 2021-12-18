# Niryo One utility Functions
import numpy as np
from numpy.linalg import inv
import math


def ForwardKinematics(t1, t2, t3, t4, t5, t6):
	# Rotation around Z axis
	tf_01 = np.array([[ math.cos(t1), -math.sin(t1),    0,   0],
					  [ math.sin(t1),  math.cos(t1),    0,   0],
					  [            0,             0,    1, 103],
					  [            0,             0,    0,   1]])
	# Rotation around Y axis
	tf_12 = np.array([[ math.cos(t2),   0, -math.sin(t2),   0],
					  [            0,   1,             0,   0],
					  [ math.sin(t2),   0,  math.cos(t2),  80],
					  [            0,   0,             0,   1]])
	# Rotation around Y axis
	tf_23 = np.array([[ math.cos(t3),   0, -math.sin(t3),   0],
					  [            0,   1,             0,   0],
					  [ math.sin(t3),   0,  math.cos(t3), 210],
					  [            0,   0,             0,   1]])
	# Rotation around X axis
	tf_34 = np.array([[   1,             0,             0,  41.5],
					  [   0,  math.cos(t4), -math.sin(t4),     0],
					  [   0,  math.sin(t4),  math.cos(t4),    30],
					  [   0,             0,             0,     1]])
	# Rotation around Y axis
	tf_45 = np.array([[  math.cos(t5),    0, math.sin(t5),   180],
					  [             0,    1,            0,     0],
					  [ -math.sin(t5),    0, math.cos(t5),     0],
					  [             0,    0,            0,     1]])
	# Rotation around X axis
	tf_56 = np.array([[   1,             0,             0,  23.7],
					  [   0,  math.cos(t6), -math.sin(t6),     0],
					  [   0,  math.sin(t6),  math.cos(t6),  -5.5],
					  [   0,             0,             0,     1]])


	tf_46 = np.matmul(tf_45, tf_56)
	#print(tf_46)
	tf_36 = np.matmul(tf_34, tf_46)
	#print(tf_36)
	tf_26 = np.matmul(tf_23, tf_36)
	#print(tf_26)
	tf_16 = np.matmul(tf_12, tf_26)
	#print(tf_16)
	tf_06 = np.matmul(tf_01, tf_16)
	#print(tf_06)

	position = np.dot(tf_06, np.transpose(np.array([0, 0, 0, 1])))
	position = position[0:3]

	return position
