import pybullet
from pybullet_simulator.env.pybullet_env import Box, Cylinder, SimRobot

# connect to simulation
pybullet.connect(pybullet.SHARED_MEMORY)

# table, objects will be placed on top of it
table = Box(basePosition=[0.3 + 0.12, 0, 0.4], baseRPY=[0, 0, 0], size=[0.6, 1.2, 0.8], useFixedBase=True)

# cylinder to place the robot
cylinder = Cylinder(basePosition=[0, 0, 0.4], baseRPY=[0, 0, 0], radius=0.1, length=0.8, rgbaColor=[0.3, 0.3, 0.3, 1],
                    useFixedBase=True)

# robot: ur5e + robotiq85
import os.path as path
robot_urdf_path = path.abspath(path.join(__file__, "../../..")) + '/robots/ur5e_robotiq85/ur5e_robotiq85.urdf'
robot = SimRobot.fromURDF(robot_urdf_path, basePosition=[0.0, 0.0, 0.8], baseRPY=[0, 0, 0], useFixedBase=True)
