import numpy as np

INIT_QPOS = {
    "xarm7": np.array([0, -0.5235, 0, 0.5235, 0, 1.0470, -0.78539] + [0] * 6),
    "panda": np.array([0, 0, 0, -1.5707963267948966, 0, 1.5707963267948966, 0.7853981633974483] + [0.0] * 2)
}

GRIPPER_NAMES = {
    "xarm7": ["left_inner_finger_pad", "right_inner_finger_pad"],
    "panda": ["panda_leftfinger", "panda_rightfinger"],
}

DRIVE_PROPERTY = {
    "xarm7": [(30000, 2000, 1000, np.arange(7)), (1000, 200, 100, np.arange(7, 9)), (50, 15, 40, np.arange(9, 13))],
    "panda": [(30000, 2000, 1000, np.arange(7)), (1000, 200, 100, np.arange(7, 9))]

}

MIMIC_JOINT_NAMES = {
    "xarm7": [("robotiq_2f_85_left_driver_joint", 1), ("robotiq_2f_85_right_driver_joint", 1),
              ("robotiq_2f_85_left_driver_mimic_joint", -10.2414), ("robotiq_2f_85_right_driver_mimic_joint", -10.2414),
              ("robotiq_2f_85_left_spring_link_joint", -10.2414), ("robotiq_2f_85_right_spring_link_joint", -10.2414)],
    "panda": [("panda_finger_joint1", 1), ("panda_finger_joint2", 1)],
}

SUPPORT_ROBOT = list(INIT_QPOS.keys())
