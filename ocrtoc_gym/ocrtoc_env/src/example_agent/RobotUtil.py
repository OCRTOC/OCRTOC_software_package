import numpy as np 
import math

def rpyxyz2H(rpy,xyz):
	Ht=[[1,0,0,xyz[0]],
	    [0,1,0,xyz[1]],
            [0,0,1,xyz[2]],
            [0,0,0,1]]

	Hx=[[1,0,0,0],
	    [0,math.cos(rpy[0]),-math.sin(rpy[0]),0],
            [0,math.sin(rpy[0]),math.cos(rpy[0]),0],
            [0,0,0,1]]

	Hy=[[math.cos(rpy[1]),0,math.sin(rpy[1]),0],
            [0,1,0,0],
            [-math.sin(rpy[1]),0,math.cos(rpy[1]),0],
            [0,0,0,1]]

	Hz=[[math.cos(rpy[2]),-math.sin(rpy[2]),0,0],
            [math.sin(rpy[2]),math.cos(rpy[2]),0,0],
            [0,0,1,0],
            [0,0,0,1]]

	Ht=np.matmul(np.matmul(np.matmul(Ht,Hz),Hy),Hx)

	return Ht



def R2axisang(R):
	ang = math.acos((R[0,0] + R[1,1] + R[2,2] - 1)/2)
	Z = np.linalg.norm([R[2,1] - R[1,2], R[0,2] - R[2,0], R[1,0] - R[0,1]])
	if Z==0:
		return[1,0,0], 0.
	x = (R[2,1] - R[1,2])/Z
	y = (R[0,2] - R[2,0])/Z
	z = (R[1,0] - R[0,1])/Z 	
	
	return[x, y, z], ang

def MatrixExp(axis,theta):

	so3_axis = so3(axis)

	R = np.eye(3) + np.sin(theta)*so3_axis + (1 - np.cos(theta))*np.matmul(so3_axis, so3_axis)

	last = np.zeros((1,4))
	last[0,3] = 1

	H_r = np.vstack((np.hstack((R,np.zeros((3,1)))),last))

	return H_r

def so3(axis):

	so3_axis = np.asarray([
		[0,-axis[2], axis[1]],
		[axis[2], 0, -axis[0]],
		[-axis[1], axis[0], 0]
		])
	return so3_axis