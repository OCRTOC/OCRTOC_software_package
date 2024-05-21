from ocrtoc_env.src.agent_base import AgentBase


# class MyAgent(AgentBase):
#     def __init__(self, target_path, **kwargs):
#         '''
#         Class that an Agent that controls the environments should be defined.
#         The Agent should inherit from the AgentBase Class.

#         Args:
#             target_path (string): path to yaml where you can find objects target poses for pose conditioned rearrangement tasks 
#                                   path to yaml where you can find target language description for language conditioned rearrangement tasks                                                           
#             kwargs (any): Additionally setting from agent_config.yml
            
#         '''
#         AgentBase.__init__(self, target_path, **kwargs)

#     def draw_action(self, observation):
#         '''
#         Fuction receive observation from enviroment and return action to control robot.
#         Control frequency: 10 Hz
  
#         Args:
#             observation (dict) {
#             "joints_state" (numpy.array,18) the first 9 are joints(joint 1-7, left gripper, right girpper) position and the last 9 are joints velocity  
#             "fix_camera_rgb_image" (numpy.array, (480, 640, 3)) the rbg image view from realsense in front of the table
#             "fix_camera_depth_image" (numpy.array, (480, 640)) the depth image view from realsense in front of the table
#             "in_hand_camera_rgb_image" (numpy.array, (480, 640, 3)) the rbg image view from realsense on panda hand
#             "in_hand_camera_depth_image" (numpy.array, (480, 640))  the depth image viewfrom realsense on panda hand
#         }

#         Returns: 
#             action (numpy.array, 8) panda arm joints position, the first 7 number are radien of joints, the last one is to control gripper force, positive value to keep it open negative value to keep it close.[-100 , 100]  
#             is_success (bool) if you finish the task, return Ture, terminate the execution and evaluate.
#         '''
#         raise NotImplementedError
   


# '''
#     Following is an example agent using keyborad to control robot
#     To play with it, you need to comment above class and uncomment the code below 
#     NOTE:when you are using keyborad agent, please set n_cores to 1
#     KEYBOARD:ws -> x 
#              ad -> y
#              qe -> z 
#              yx -> roll
#              12 -> pitch
#              34 -> yaw
#              c -> close gripper 
#              o -> open gripper
#              m -> task done 
# '''

from ocrtoc_env.src.example_agent.keyborad_agent import KeyboradAgent
class MyAgent(KeyboradAgent):
    def __init__(self, target_path, **kwargs):
        KeyboradAgent.__init__(self, target_path, **kwargs)
