import numpy as np
import ocrtoc_env.src.example_agent.RobotUtil as rt
import math
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D


class FrankArm:

	def __init__(self):
		# Robot descriptor taken from URDF file (rpy xyz for each rigid link transform) - NOTE: don't change
		self.Rdesc=[
		[0, 0, 0, 0., 0, 0.333], # From robot base to joint1
		[-np.pi/2, 0, 0, 0, 0, 0],
		[np.pi/2, 0, 0, 0, -0.316, 0],
		[np.pi/2, 0, 0, 0.0825, 0, 0],
		[-np.pi/2, 0, 0, -0.0825, 0.384, 0],
		[np.pi/2, 0, 0, 0, 0, 0],
		[np.pi/2, 0, 0, 0.088, 0, 0],
		[0, 0, 0, 0, 0, 0.107] # From joint5 to end-effector center
		]

		#Define the axis of rotation for each joint 
		self.axis=[
		[0, 0, 1],
		[0, 0, 1],
		[0, 0, 1],
		[0, 0, 1],
		[0, 0, 1],
		[0, 0, 1],
		[0, 0, 1],
		[0, 0, 1]
		]

		#Set base coordinate frame as identity - NOTE: don't change
		self.Tbase= [[1,0,0,0],
		[0,1,0,0],
		[0,0,1,0],
		[0,0,0,1]]

		#Initialize matrices - NOTE: don't change this part
		self.Tlink=[] #Transforms for each link (const)
		self.Tjoint=[] #Transforms for each joint (init eye)
		self.Tcurr=[] #Coordinate frame of current (init eye)
		for i in range(len(self.Rdesc)):
			self.Tlink.append(rt.rpyxyz2H(self.Rdesc[i][0:3],self.Rdesc[i][3:6]))
			self.Tcurr.append([[1,0,0,0],[0,1,0,0],[0,0,1,0.],[0,0,0,1]])
			self.Tjoint.append([[1,0,0,0],[0,1,0,0],[0,0,1,0.],[0,0,0,1]])

		self.Tlink = np.array(self.Tlink)
		self.Tcurr = np.array(self.Tcurr)
		self.Tjoint = np.array(self.Tjoint)
		self.Tlinkzero=rt.rpyxyz2H(self.Rdesc[0][0:3],self.Rdesc[0][3:6])  

		self.Tlink[0]=np.matmul(self.Tbase,self.Tlink[0])					

		# initialize Jacobian matrix
		self.J=np.zeros((6,7))

		self.q=[0.,0.,0.,0.,0.,0.,0.]
		self.ForwardKin([0.,0.,0.,0.,0.,0.,0.])

	def ForwardKin(self,ang):
		'''
		inputs: joint angles
		outputs: joint transforms for each joint, Jacobian matrix
		'''

		self.q[0:-1]=ang

		# Compute current joint and end effector coordinate frames (self.Tjoint). 
		# Remember that not all joints rotate about the z axis!
		for i in range(len(self.q)):
			self.Tjoint[i] = np.array([
				[np.cos(self.q[i]), -np.sin(self.q[i]), 0, 0],
				[np.sin(self.q[i]), np.cos(self.q[i]), 0, 0],
				[ 0, 0, 1, 0],
				[0, 0, 0, 1]
			])
			if i == 0:
				self.Tcurr[i] = self.Tlink[i] @ self.Tjoint[i]
			else:
				self.Tcurr[i] = self.Tcurr[i-1] @ self.Tlink[i] @ self.Tjoint[i]

		for i in range(len(self.Tcurr) - 1):
			p = self.Tcurr[-1][0:3, 3] - self.Tcurr[i][0:3, 3]
			a = self.Tcurr[i][0:3, 2]
			self.J[:3, i] = np.cross(a, p)
			self.J[3:7, i] = a

		return self.Tcurr, self.J



	def IterInvKin(self,ang,TGoal,x_eps=1e-3, r_eps=1e-3):
		'''
		inputs: starting joint angles (ang), target end effector pose (TGoal)

		outputs: computed joint angles to achieve desired end effector pose, 
		Error in your IK solution compared to the desired target
		'''	

		W = np.eye(7)
		C = np.eye(6)
		C[0, 0] = 1000000.0
		C[1, 1] = 1000000.0
		C[2, 2] = 1000000.0
		C[3, 3] = 1000.0
		C[4, 4] = 1000.0
		C[5, 5] = 1000.0

		W[2, 2] =100.0
		W[3, 3] =100.0
		W[-1, 0] = 1
		W[-1, -1] =100.0
		Winv = np.linalg.inv(W)

		Err = np.zeros((6,))
		xErr, rErr = 10, 10
		# print(np.array(ang).shape, np.array(self.q).shape)
		self.q[:7] = ang
		self.ForwardKin(ang)
		# exit()
		while np.linalg.norm(xErr) > x_eps or np.linalg.norm(rErr) > r_eps:
			rErrR = TGoal[:3, :3] @ self.Tcurr[-1][:3, :3].T
			rErrAxis, rErrAngle = rt.R2axisang(rErrR)

			
			rErrAngle = np.clip(rErrAngle, -0.1, 0.1)
			rErr = np.array(rErrAxis) * rErrAngle
			# print(np.linalg.norm(xErr), np.linalg.norm(rErr))

			xErr = TGoal[:3, 3] - self.Tcurr[-1][:3, 3]

			if np.linalg.norm(xErr) > 0.01:
				xErr *= 0.01/np.linalg.norm(xErr)

			Err[:3] = xErr
			Err[3:] = rErr

			
			Jhash = Winv @ self.J.T @ np.linalg.inv(self.J @ Winv @ self.J.T + np.linalg.inv(C))
			delta = Jhash @ Err
			self.q[0:7] += delta
			# print(np.linalg.norm(delta))
			self.ForwardKin(self.q[:7])


		return self.q[0:-1], Err