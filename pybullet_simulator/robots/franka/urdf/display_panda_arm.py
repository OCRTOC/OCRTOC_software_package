#!/usr/bin/env python

# import sys
# import argparse
# parser = argparse.ArgumentParser(usage='Load an urdf file')
# parser.add_argument('urdf_name', type=str, help='pass urdf name')
# args = parser.parse_args()
# print("urdf_name:", args.urdf_name)

import os
urdf_path = os.getcwd() + "/" + 'panda_arm.urdf'
print("urdf_path:", urdf_path)

import numpy as np
import pybullet
pybullet.connect(pybullet.GUI)
pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 1)
pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

robot = pybullet.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)
pybullet.setGravity(0, 0, -9.81)
jointIds = []
paramIds = []

pybullet.changeDynamics(robot, -1, linearDamping=0, angularDamping=0)
dof = pybullet.getNumJoints(robot)
jointAngles = [0] * dof
# jointAngles=[0,0,1.0204,-1.97,-0.084,2.06,-1.9,0,0,1.0204,-1.97,-0.084,2.06,-1.9,0]
activeJoint = 0
for j in range(pybullet.getNumJoints(robot)):
    pybullet.changeDynamics(robot, j, linearDamping=0, angularDamping=0)
    info = pybullet.getJointInfo(robot, j)
    print(info)
    jointName = info[1]
    jointType = info[2]
    jointLowerLimit = info[8]
    jointUpperLimit = info[9]
    linkName = info[12]
    if (jointType == pybullet.JOINT_PRISMATIC or jointType == pybullet.JOINT_REVOLUTE):
        jointIds.append(j)
        paramIds.append(pybullet.addUserDebugParameter(jointName.decode("utf-8"), jointLowerLimit, jointUpperLimit, jointAngles[activeJoint]))
        pybullet.resetJointState(robot, j, jointAngles[activeJoint])
        activeJoint += 1

length = 0.2
width = 2
for i in range(pybullet.getNumJoints(robot)):
    print(i)
    pybullet.addUserDebugLine([0, 0, 0], [length, 0, 0], [1, 0, 0], lineWidth=width, parentObjectUniqueId=robot, parentLinkIndex=pybullet.getJointInfo(robot, i)[-1])
    pybullet.addUserDebugLine([0, 0, 0], [0, length, 0], [0, 1, 0], lineWidth=width, parentObjectUniqueId=robot, parentLinkIndex=pybullet.getJointInfo(robot, i)[-1])
    pybullet.addUserDebugLine([0, 0, 0], [0, 0, length], [0, 0, 1], lineWidth=width, parentObjectUniqueId=robot, parentLinkIndex=pybullet.getJointInfo(robot, i)[-1])
    print(pybullet.getJointInfo(robot, i)[12].decode("utf-8"))
    pybullet.addUserDebugText(pybullet.getJointInfo(robot, i)[12].decode("utf-8"), np.random.random(3)*0.01, textColorRGB=[0, 0, 0], textSize=2, parentObjectUniqueId=robot,  parentLinkIndex=pybullet.getJointInfo(robot, i)[-1])



pybullet.setRealTimeSimulation(1)
while True:
    for i in range(len(paramIds)):
        c = paramIds[i]
        targetPos = pybullet.readUserDebugParameter(c)
        pybullet.setJointMotorControl2(robot, jointIds[i], pybullet.POSITION_CONTROL, targetPos)

