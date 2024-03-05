import numpy as np

class AgentBase():
    def __init__(self, target_path, **kwargs):
        self.is_success = False
        self.target_path = target_path
        
    def draw_action(self, observation):
        action = np.zeros(8)
       
        return action , self.is_success
