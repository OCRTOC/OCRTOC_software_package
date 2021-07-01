#!/usr/bin/env python
import io
import yaml

import os.path as path
import numpy as np

panda_configs = {'base_position': [-0.42, 0.0, 0.04],
                 'base_rpy': [0, 0, 0],
                 'arm_joint_names': ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5',
                                     'panda_joint6', 'panda_joint7'],
                 'arm_link_names': ['panda_link1', 'panda_link2', 'panda_link3', 'panda_link4', 'panda_link5',
                                    'panda_link6', 'panda_link7'],
                 'gripper_joint_names': ['panda_finger_joint1', 'panda_finger_joint2'],
                 'gripper_link_names': ['panda_leftfinger', 'panda_rightfinger'],
                 'tool_frame_name': 'tool',
                 'arm_home_config': {'panda_joint1': 0,
                                     'panda_joint2': 0,
                                     'panda_joint3': 0,
                                     'panda_joint4': float(np.deg2rad(-90)),
                                     'panda_joint5': 0,
                                     'panda_joint6': float(np.deg2rad(90)),
                                     'panda_joint7': float(np.deg2rad(45))},
                 'gripper_home_config': {'panda_finger_joint1': 0, 'panda_finger_joint2': 0},
                 'fixed_base':1
                 }

# Write YAML file
with io.open('panda_arm_hand.yaml', 'w', encoding='utf8') as outfile:
    yaml.dump(panda_configs, outfile, default_flow_style=False, allow_unicode=True)


# Read YAML file
with open("panda_arm_hand.yaml", 'r') as stream:
    data_loaded = yaml.safe_load(stream)

print(data_loaded)