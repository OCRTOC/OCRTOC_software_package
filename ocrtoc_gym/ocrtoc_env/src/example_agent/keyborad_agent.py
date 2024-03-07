import numpy as np
import mujoco
from ocrtoc_env.src.example_agent.Franka import FrankArm
import sys   
import termios
import tty
from select import select
from scipy.spatial.transform import Rotation as R
from ocrtoc_env.src.agent_base import AgentBase


class KeyboradAgent(AgentBase):
    def __init__(self, target_path, **kwargs):
        AgentBase.__init__(self, target_path, **kwargs)

        self.mybot=FrankArm()
        self.H_pos_x = 0.35
        self.H_pos_y = 0
        self.H_pos_z = 0.3 
        self.gripper = 0
        self.r = R.from_matrix([[1, 0, 0],
                   [0, -1, 0],
                   [0, 0, -1]])
        self.euler = self.r.as_euler('zyx', degrees=True)
        self.HGoal= np.array([[1.,0.,0.,self.H_pos_x], # target EE pose
                [0.,-1.,0.,self.H_pos_y],
                [0.,0,-1.,self.H_pos_z],
                [0.,0.,0.,1]])
        
    def draw_action(self, observation):
        action = np.zeros(8)
        qInit = observation['joints_state'][0:7]
        key = self.keyboard() 
        if key is not False:
            self.H_pos_x = self.H_pos_x + key[0] *0.02
            self.H_pos_y = self.H_pos_y + key[1] * 0.02 
            self.H_pos_z = self.H_pos_z + key[2] * 0.02
            self.euler[0] = self.euler[0] + key[3] * 10 
            new_r = R.from_euler("zyx",self.euler,degrees=True)
            new_rotation_matrix = new_r.as_matrix()
            self.HGoal[0:3,0:3] = new_rotation_matrix
            self.HGoal[0,3] = self.H_pos_x
            self.HGoal[1,3] = self.H_pos_y
            self.HGoal[2,3] = self.H_pos_z
        self.q,Err=self.mybot.IterInvKin(qInit, self.HGoal)
        action[0:7] = self.q
        action[7] = self.gripper
        return action , self.is_success

    def keyboard(self):
        settings = self.saveTerminalSettings()
        key_timeout = 0.05
        key = self.getKey(settings, key_timeout)
        if key == "m":
            self.is_success = True
        if key == "o":
            self.gripper = 1
        if key == "c":
            self.gripper = -1
        if key == 'w':
            return np.array([1,0,0,0])
        elif key == 's':
            return np.array([-1,0,0,0])
        elif key == 'a':
            return np.array([0,1,0,0])
        elif key == 'd':
            return np.array([0,-1,0,0])      
        elif key == 'q':
            return np.array([0,0,1,0])        
        elif key == 'e':
            return np.array([0,0,-1,0])   
        elif key == 'y':
            return np.array([0,0,0,1])  
        elif key == 'x':
            return np.array([0,0,0,-1])  
        else:
            return False
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


