import pybullet
import numpy as np
from pybullet_simulator.env.pybullet_env import SimEnv, SimRobot, Manipulator, Kinect

# connect to simulation
physicsClientId = pybullet.connect(pybullet.SHARED_MEMORY)
if physicsClientId == -1:
    print('kinect failed to connect the physics server!')
else:
    print('kinect connected to the physics server:', physicsClientId)

# kinect
import os.path as path
kinect_urdf_path = path.abspath(path.join(__file__, "../../..")) + '/robots/kinect/kinect.urdf'

robot_base_position = np.array([-0.42, 0.0, 0.04])
kinect_base_position = robot_base_position + np.array([1.1361064293274783, 0.006070980252324755,  0.6919819764614929])
kinect = Kinect.fromURDF(kinect_urdf_path, basePosition=kinect_base_position, baseRPY=[0, 0, np.pi], useFixedBase=True)




