from ocrtoc_env.src.agent_base import AgentBase


# class MyAgent(AgentBase):
#     def __init__(self, target_path, **kwargs):
#         '''
#         Class that an Agent that controls the environments should be defined.
#         The Agent should inherit from the AgentBase Class.

#         Args:
#             target_path (string): path to yaml where you can find objects target poses
#             kwargs (any): Additionally setting from agent_config.yml
            
#         '''
#         AgentBase.__init__(self, env_model, target_path, **kwargs)

#     def draw_action(self, observation):
#         '''
#         Fuction receive observation from enviroment and return action to control robot.

#         Args:
#             observation (dict) {
#             "joints_state" (numpy.array,18) the first 9 are joints(joint 1-7, left gripper, right girpper) position and the last 9 are joints velocity  
#             "realsense_rgb_image" (numpy.array, (720, 1280, 3)) the rbg image view from realsense on panda hand
#             "realsense_depth_image" (numpy.array, (720, 1280)) the depth image view from realsense on panda hand
#             "realsense2_rgb_image" (numpy.array, (720, 1280, 3)) the rbg image view from realsense in front of the table
#             "realsense2_rgb_image" (numpy.array, (720, 1280))  the rbg image view from realsense in front of the table
#         }

#         Returns: 
#             action (numpy.array, 8) panda arm joints position, the first 7 number are radien of joints, the last one is to control gripper, 1 to keep it open -1 to keep it close.  
#             is_success (bool) if you finish the task, return Ture, terminate the execution and evaluate.
#         '''
#         raise NotImplementedError
   


'''
    Following is an example agent using keyborad to control robot
    To play with it, you need to comment above class and uncomment the code below 
    
    KEYBOARD: w -> +x 
              s -> -x
              a -> +y
              d -> -y 
              q -> +z 
              e -> -z
              y -> +yaw
              x -> -yaw
              o -> open gripper
              c -> close gripper 
              m -> task done 
'''

from ocrtoc_env.src.example_agent.keyborad_agent import KeyboradAgent
class MyAgent(KeyboradAgent):
    def __init__(self, target_path, **kwargs):
        KeyboradAgent.__init__(self, target_path, **kwargs)
