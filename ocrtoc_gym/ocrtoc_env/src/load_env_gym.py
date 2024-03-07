import mujoco
import numpy as np
from gymnasium.core import ObsType
from gymnasium_robotics.envs.robot_env import MujocoRobotEnv
# from gymnasium_robotics.utils import rotations
from typing import Optional, Any, SupportsFloat
from matplotlib import pyplot as plt
import math
# import open3d as o3d

DEFAULT_CAMERA_CONFIG = {
    "distance": 2.5,
    "azimuth": 135.0,
    "elevation": -20.0,
    "lookat": np.array([0.0, 0.0, 0.0]),

}


class FrankaEnv(MujocoRobotEnv):
    metadata = {
        "render_modes": [
            "human",
            "rgb_array",
        ],
        "render_fps": 50,
    }

    def __init__(
        self,
        model_path: str = "/home/zhen/OCRTOC_software_package/ocrtoc_materials_mujoco/scenes/1-1-1.xml",
        n_substeps: int = 10,
        **kwargs,
    ):
        self.model_path = model_path

        action_size = 8

        # robot home pose
        self.neutral_joint_values = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.78, 0.0, 0.0])

        # camera view size 
        self.width = 1280
        self.height = 720
        # self.intrinsics =  o3d.camera.PinholeCameraIntrinsic(self.width,self.height,np.array(self.calIntrinsicMatrix()).reshape((3, 3)))
        super().__init__(
            n_actions=action_size,
            n_substeps=n_substeps,
            model_path=self.model_path,
            initial_qpos=self.neutral_joint_values,
            default_camera_config=DEFAULT_CAMERA_CONFIG,
            **kwargs,
        )     
    
    # overwrite initialize function
    def _initialize_simulation(self):
        self.model = self._mujoco.MjModel.from_xml_path(self.fullpath)
        self.data = self._mujoco.MjData(self.model)
        self._model_names = self._utils.MujocoModelNames(self.model)
        self.model.vis.global_.offwidth = self.width
        self.model.vis.global_.offheight = self.height

        # index used to distinguish arm and gripper joints
        self.arm_joint_names = self._model_names.joint_names[0:7]
        self.gripper_joint_names = self._model_names.joint_names[7:9]
        self._env_setup(self.neutral_joint_values)
        self.initial_time = self.data.time
        self.initial_qvel = np.copy(self.data.qvel)
        from gymnasium.envs.mujoco.mujoco_rendering import MujocoRenderer

        self.mujoco_renderer = MujocoRenderer(
            self.model, self.data
        )

    # set env
    def _env_setup(self, neutral_joint_values):
        self.data.ctrl[0:7] = neutral_joint_values[0:7]
        self._mujoco.mj_forward(self.model, self.data)

    # render 
    def render_images(self):
        rgb_image = self.mujoco_renderer.render("rgb_array", camera_name="realsense_rgb")
        depth_image = self.mujoco_renderer.render("depth_array", camera_name="realsense_depth")
        rgb_image_kinect = self.mujoco_renderer.render("rgb_array", camera_name="realsense2_rgb")
        depth_image_kinect = self.mujoco_renderer.render("depth_array", camera_name="realsense2_depth")
        return rgb_image , depth_image, rgb_image_kinect, depth_image_kinect
    
    # each simulation step
    def step(self, action):
        if np.array(action).shape != self.action_space.shape:
            raise ValueError("Action dimension mismatch")
        # action = np.clip(action, self.action_space.low, self.action_space.high)
        self._set_action(action)

        self._mujoco_step(action)
      
        self.render()
        
        self._step_callback()

        # visualize depth image 
        # depth = self.depth_image
        # plt.imshow(depth, cmap="plasma")
        # plt.show()

        # visualize color image
        # plt.imshow(self.rgb_image_kinect)
        # plt.show()

        # # visualize point cloud 
        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(self.point_cloud)
        # o3d.visualization.draw_geometries([self.point_cloud])

        obs = self._get_obs().copy()

        info = {}
        terminated = False
        truncated = False
        reward = 0

        return obs, reward, terminated, truncated, info

    # not used
    def compute_reward(self, achieved_goal, desired_goal, info):

            return 0.0

    # set action to controller
    def _set_action(self, action):
        self.data.ctrl[0:7] = action[0:7]
        if action[7] == 1: 
            self.data.qfrc_applied[7] = 50
        elif action[7] == -1:
            self.data.qfrc_applied[7] = -50
        else:
            self.data.qfrc_applied[7]  = 0

    # get data from simulator
    def _get_obs(self):
        joints_pos = np.array([])
        joints_vel = np.array([])
        for joint in self._model_names.joint_names:
            joints_pos = np.append(joints_pos,self._utils.get_joint_qpos(self.model, self.data, joint))
            joints_vel = np.append(joints_vel,self._utils.get_joint_qvel(self.model, self.data, joint))
        joints_state = np.concatenate((joints_pos.copy(), joints_vel.copy()))

        self.rgb_image, self.depth_image, self.rgb_image_kinect ,self.depth_image_kinect= self.render_images()
        self.depth_image = self.depth_image_scale(self.depth_image)
        self.depth_image_kinect = self.depth_image_scale(self.depth_image_kinect)

        # calculate point cloud
        # self.point_cloud = self.img_to_pointcloud(self.rgb_image, self.depth_image)
        # self.point_cloud_kinect = self.img_to_pointcloud(self.rgb_image_kinect, self.depth_image_kinect)

        obs = {
            "joints_state":joints_state.copy(),
            "realsense_rgb_image":self.rgb_image.copy(),
            "realsense_depth_image":self.depth_image.copy(),
            "realsense2_rgb_image":self.rgb_image_kinect.copy(),
            "realsense2_depth_image":self.depth_image_kinect.copy()
        }
        return obs

    # not used
    def _is_success(self, achieved_goal, desired_goal):
        return 1.0 

    # not used
    def _render_callback(self):
        pass

    # reset env
    def _reset_sim(self):
        self.data.time = self.initial_time
        self.data.qvel[:] = np.copy(self.initial_qvel)
        self.data.qpos = self.model.key_qpos[0]
        self._mujoco.mj_forward(self.model, self.data)
        return True
    # mujoco simulation step
    def _mujoco_step(self, action: Optional[np.ndarray] = None):
        self._mujoco.mj_step(self.model, self.data, nstep=self.n_substeps)

    # not used
    def goal_distance(self, goal_a, goal_b):
        assert goal_a.shape == goal_b.shape
        return np.linalg.norm(goal_a - goal_b, axis=-1)

    # robot home pose
    def set_joint_neutral(self):
        # assign value to arm joints
        for name, value in zip(self.arm_joint_names, self.neutral_joint_values[0:7]):
            self._utils.set_joint_qpos(self.model, self.data, name, value)

        # assign value to finger joints
        for name, value in zip(self.gripper_joint_names, self.neutral_joint_values[7:9]):
            self._utils.set_joint_qpos(self.model, self.data, name, value)
    
    # not used
    def _sample_goal(self):
        goal = np.array([0.0, 0.0, 0.1])    

        return goal
    
    # def calIntrinsicMatrix(self):
    #         f = math.sqrt(self.width * self.width / 4.0 + self.height * self.height / 4.0) / 2.0 / math.tan(self.fov / 2.0 / 180.0 * math.pi)
    #         return (f, 0.0, self.width / 2.0 - 0.5, 0.0, f, self.height / 2.0 - 0.5, 0.0, 0.0, 1.0)
    
    #rescale depth image
    def depth_image_scale(self,depth_img):
        extent = self.model.stat.extent
        near = self.model.vis.map.znear * extent
        far = self.model.vis.map.zfar * extent
        scaled_depth_img = near / (1 - depth_img * (1 - near / far))
        depth_img = scaled_depth_img.squeeze()
        return depth_img
    
    # def img_to_pointcloud(self, rgb, depth):
    #     im = o3d.geometry.Image(rgb.reshape(self.height,self.width,3).astype("uint8"))
    #     rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(im, o3d.geometry.Image(depth), depth_scale = 1 ,convert_rgb_to_intensity = False)
    #     pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.intrinsics)
    #     return np.asarray(pcd.points)
