import gymnasium as gym
from gymnasium import spaces
import numpy as np
import mujoco
import mujoco.viewer
import time

class FrankaEnv(gym.Env):
    '''
        load Mujoco model and interface it with Gymnasium
    '''
    
    metadata = {"render_modes": ["human", "rgb_array"]}
    def __init__(
        self,
        model_path: str = "/home/zhen/OCRTOC_software_package/ocrtoc_materials_mujoco/scenes/1-1-1.xml",
        n_substeps: int = 50,
        render_mode = None,
        **kwargs,
    ):
        # gymnasium metadata
        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self.render_mode = render_mode  

        # Mujoco parameters
        self.model_path = model_path # mujoco model path
        self.gravity_compensation = True  # Whether to enable gravity compensation.
        self.dt = 0.1 # Simulation timestep in seconds.
        self.nstep = n_substeps # mujoco substep
        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, 480,640)
        
        # Robot variables
        self.neutral_joint_values = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.78, 0.0, 0.0]) # robot home pose
        self.initial_qvel = np.copy(self.data.qvel) # init vel
        self.action_size = 8
        ## Gravity compensation
        self.model.body_gravcomp = 0.0  
        body_names = ['hand', 'left_finger', 'link0', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7', 'right_finger',"d435i","d435i_link","d435i2","d435i2_link","fix_camera_depth_frame","fix_camera_rgb_frame","in_hand_camera_depth_frame","in_hand_camera_rgb_frame"]         # Name of bodies we wish to apply gravity compensation to.
        body_ids = [self.model.body(name).id for name in body_names]
        if self.gravity_compensation:
            self.model.body_gravcomp[body_ids] = 1.0
        self.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
        ]
        self.actuator_names = [
            "actuator1",
            "actuator2",
            "actuator3",
            "actuator4",
            "actuator5",
            "actuator6",
            "actuator7",
        ]
        self.joint_names_with_gripper = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
        ]
        self.dof_ids = np.array([self.model.joint(name).id for name in self.joint_names])
        self.dof_ids_with_gripper = np.array([self.model.joint(name).id for name in self.joint_names_with_gripper])

        self.actuator_ids = np.array([self.model.actuator(name).id for name in self.actuator_names])
        self.gripper_actuator_ids = self.model.actuator("actuator8").id

        # init
        ## init GUI
        if self.render_mode == "human":
            self.viewer =  mujoco.viewer.launch_passive(model=self.model, data=self.data, show_left_ui=True, show_right_ui=False) 
        ## init robot 
        self.data.qpos[0:7] = self.neutral_joint_values[0:7].copy()
        mujoco.mj_forward(self.model, self.data)
        if self.render_mode == "human":
            self.viewer.sync()
            mujoco.mjv_defaultFreeCamera(self.model, self.viewer.cam)# Initialize the camera view to that of the free camera.
        
        # Define obs, action space
        observation = self._get_obs()
        self.action_space = spaces.Box(-1.0, 1.0, shape=(self.action_size,), dtype="float32")
        self.observation_space = spaces.Dict(
            dict(
                joints_state=spaces.Box(
                    -np.inf, np.inf, shape=observation["joints_state"].shape, dtype="float64"
                ),
                fix_camera_rgb_image=spaces.Box(
                    0, 255, shape=observation["fix_camera_rgb_image"].shape, dtype="uint8"
                ),
                fix_camera_depth_image=spaces.Box(
                    -np.inf, np.inf, shape=observation["fix_camera_depth_image"].shape, dtype="float32"
                ),
                in_hand_camera_rgb_image=spaces.Box(
                    0, 255, shape=observation["in_hand_camera_rgb_image"].shape, dtype="uint8"
                ),
                in_hand_camera_depth_image=spaces.Box(
                    -np.inf, np.inf, shape=observation["in_hand_camera_depth_image"].shape, dtype="float32"
                ),
            )
        )

    # reset env
    def reset(self, seed=None, options=None):
        self.data.qvel[:] = np.copy(self.initial_qvel)
        self.data.qpos = self.model.key_qpos[0]
        mujoco.mj_forward(self.model, self.data)
        if self.render_mode == "human":
            self.viewer.sync()
        observation = self._get_obs()
        info = {}
        return observation,info 

    # set action to mujoco
    def _set_action(self,action):
        self.data.ctrl[self.actuator_ids] = action[self.dof_ids]
        self.data.qfrc_applied[self.gripper_actuator_ids] = action[-1]
    
    # get data from simulator
    def _get_obs(self):
        joints_pos = np.array([])
        joints_vel = np.array([])
        joints_pos = np.append(joints_pos,self.data.qpos.copy()[self.dof_ids_with_gripper])
        joints_vel = np.append(joints_vel,self.data.qvel.copy()[self.dof_ids_with_gripper])
        joints_state = np.concatenate((joints_pos.copy(), joints_vel.copy()),dtype=np.float64)
        self._render_frame()            
        obs = {
            "joints_state":joints_state.copy(),
            "fix_camera_rgb_image":self.fix_camera_rgb_image.copy(),
            "fix_camera_depth_image":self.fix_camera_depth_image.copy(),
            "in_hand_camera_rgb_image":self.in_hand_camera_rgb_image.copy(),
            "in_hand_camera_depth_image":self.in_hand_camera_depth_image.copy()
        }
        return obs

    # Gymnasium step frequence 10 Hz
    def step(self, action):
        step_start = time.time()
        self._set_action(action)
        mujoco.mj_step(self.model, self.data, nstep=self.nstep) 
        if self.render_mode == "human":
            self.viewer.sync()
        observation = self._get_obs().copy()
        info = {}
        terminated = False
        truncated = False
        reward = 0
        time_until_next_step = self.dt - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
        return observation , reward, terminated, truncated, info
    

    # rende images
    def _render_frame(self):
        self.renderer._depth_rendering = False
        self.renderer.update_scene(self.data, camera="fix_camera_rgb")
        self.fix_camera_rgb_image = self.renderer.render()
        self.renderer._depth_rendering = True
        self.renderer.update_scene(self.data, camera="fix_camera_depth")
        self.fix_camera_depth_image = self.renderer.render()
        self.renderer._depth_rendering = False
        self.renderer.update_scene(self.data, camera="in_hand_camera_rgb")
        self.in_hand_camera_rgb_image = self.renderer.render()
        self.renderer._depth_rendering = True
        self.renderer.update_scene(self.data, camera="in_hand_camera_depth")
        self.in_hand_camera_depth_image = self.renderer.render()