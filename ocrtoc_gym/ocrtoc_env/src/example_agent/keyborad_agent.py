import numpy as np
import sys   
import termios
import tty
import os
from select import select
from ocrtoc_env.src.agent_base import AgentBase
from numpy.linalg import pinv
import kinpy as kp


class KeyboradAgent(AgentBase):
    def __init__(self, target_path, **kwargs):
        AgentBase.__init__(self, target_path, **kwargs)
        ## load urdf and calculate diffential inverse kinematic
        self.urdf_path = os.path.dirname(os.path.realpath(__file__)) + "/panda_arm.urdf"
        self.chain = kp.build_serial_chain_from_urdf(open(self.urdf_path).read(), "panda_link8")
        self.jac = np.zeros((6, 7))

        # initial 
        self.is_success = False
        self.target_joints = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.78])
        self.gripper_target = 0

    # draw action and return to env
    def draw_action(self, observation):
        action = np.zeros(8)
        qInit = observation['joints_state'][0:7]
        self.jac = self.cal_jacobian(qInit)
        key = self.keyboard() 
        if len(key) == 1: 
            self.gripper_target = key[0]
        else:
            target_joint_velocity = pinv(self.jac) @ key*0.1
            self.target_joints = self.target_joints + target_joint_velocity*0.1
        action[0:7] = self.target_joints
        action[7] = self.gripper_target
        return action , self.is_success
    
    # calculate jacobian matrix
    def cal_jacobian(self,th):
        self.jac = self.chain.jacobian(th)
        return self.jac

    # keyboard interface
    def keyboard(self):
        settings = self.saveTerminalSettings()
        key_timeout = 0.05
        key = self.getKey(settings, key_timeout)
        if key == "m":
            self.is_success = True
        if key == "o":
            return np.array([30])
        if key == "c":
            return np.array([-30])
        if key == 'w':
            return np.array([1,0,0,0,0,0])
        elif key == 's':
            return np.array([-1,0,0,0,0,0])
        elif key == 'a':
            return np.array([0,1,0,0,0,0])
        elif key == 'd':
            return np.array([0,-1,0,0,0,0])      
        elif key == 'q':
            return np.array([0,0,1,0,0,0])        
        elif key == 'e':
            return np.array([0,0,-1,0,0,0])   
        elif key == 'y':
            return np.array([0,0,0,1,0,0]) * 5  
        elif key == 'x':
            return np.array([0,0,0,-1,0,0]) * 5  
        elif key == '1':
            return np.array([0,0,0,0,1,0]) * 5  
        elif key == '2':
            return np.array([0,0,0,0,-1,0]) * 5  
        elif key == '3':
            return np.array([0,0,0,0,0,1]) * 5  
        elif key == '4':
            return np.array([0,0,0,0,0,-1]) * 5  
        else:
            return np.array([0,0,0,0,0,0]) 
    def saveTerminalSettings(self):
            if sys.platform == 'win32':
                return None
            return termios.tcgetattr(sys.stdin)

    def getKey(self,settings, timeout):
            if sys.platform == 'win32':
                # getwch() returns a string on Windows
                key = msvcrt.getwch()
            else:
                tty.setraw(sys.stdin.fileno())
                # sys.stdin.read() returns a string on Linux
                rlist, _, _ = select([sys.stdin], [], [], timeout)
                if rlist:
                    key = sys.stdin.read(1)
                else:
                    key = ''
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            return key


