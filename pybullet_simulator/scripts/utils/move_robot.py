import time
import numpy as np
import pybullet
from env.pybullet_env import Box, Cylinder
from env.pybullet_env import SimRobot, Manipulator

# connect to simulation
pybullet.connect(pybullet.SHARED_MEMORY)



panda_config_path = '/home/xin/codes/OCRTOC-PyBullet/robots/franka/config/panda_arm_hand.yaml'



robot = Manipulator.loadFromID(id=1, config_path=panda_config_path)



# robot.gotoArmJointConfig([0,0,0,np.deg2rad(-90),0,np.deg2rad(90),np.deg2rad(45)])
# print([0,0,0,np.deg2rad(-90),0,np.deg2rad(90),np.deg2rad(45)])

robot.goHome()
# robot.goZero()

# robot.gotoCartesianTarget(position=[0.2, 0.15, 0.1], rpy=[-np.pi ,0 ,-np.pi /2], T=1.0)

# robot.gotoCartesianTarget(position=[0.2, 0.15, 0.07], rpy=[-np.pi ,0 ,-np.pi /2], T=1.0)

robot.gripperControl(gripper_opening_length=0.049, T=1.0)
robot.gripperControl(gripper_opening_length=0.0, T=1.0)


# robot.gotoCartesianTarget(position=[0.2, 0.15, 0.2], rpy=[-np.pi ,0 ,-np.pi /2], T=1.0)

# robot.gotoCartesianTarget(position=[0.0, 0.3, 0.2], rpy=[-np.pi ,0 ,-np.pi /2], T=1.0)

# robot.gripperControl(gripper_opening_length=0.052, T=1.0)
