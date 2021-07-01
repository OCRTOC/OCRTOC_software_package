#!/usr/bin/python
from pybullet_env import SimEnv, SimRobot, Manipulator, Camera
from pybullet_env import Box, Cylinder
import numpy as np
import pybullet
import os

from gazebo_ros.gazebo_interface import GetModelStateResponse
from rosgraph_msgs.msg import Clock
import rospkg
import rospy

def get_model_id_name_map():
    num_bodies = pybullet.getNumBodies()
    id_name_map = {}
    for num_body in range(num_bodies):
        body_name = pybullet.getBodyInfo(num_body)[1]
        id_name_map[num_body] = body_name
    return id_name_map

def handle_get_model_state(req):
    # print('get_model_state service requested:', req)
    model_name = req.model_name
    id_name_map = get_model_id_name_map()
    # print('avaliable model names:', id_name_map.values())
    if model_name in id_name_map.values():
        model_id = id_name_map.keys()[id_name_map.values().index(model_name)]
        # for object_id in scene_object_ids:
        pose = pybullet.getBasePositionAndOrientation(model_id)
        # print('object_id:', int(req.model_name), pose)
        res = GetModelStateResponse()
        res.header.stamp = rospy.Time.now()
        res.pose.position.x = pose[0][0]
        res.pose.position.y = pose[0][1]
        res.pose.position.z = pose[0][2] - 0.04
        res.pose.orientation.x = pose[1][0]
        res.pose.orientation.y = pose[1][1]
        res.pose.orientation.z = pose[1][2]
        res.pose.orientation.w = pose[1][3]
        res.success = True
        res.status_message = 'model id: ' + str(model_id)
    else:
        rospy.logerr('model_name error!')
        res = GetModelStateResponse()
        res.success = False
        res.status_message = 'model_name error!'

    return res

if __name__ == '__main__':
    #sim_time_publisher = rospy.Publisher('/clock', Clock, queue_size=10)
    rospy.init_node('pybullet_env', anonymous=True)
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('pybullet_simulator')

    sim_env = SimEnv(sim_rate=1000, GUI=True)
    sim_env.resetGUIView(distance=0.8, yaw=90, pitch=-60, target_position=[0,0,0.0])
    sim_env.resetLightPosition(lightPosition=[1,0,1])

    # panda arm
    panda_urdf_path = os.path.join(pkg_path, 'robots/franka/urdf/panda_arm_hand_realsense.urdf')
    panda_config_path = os.path.join(pkg_path, 'robots/franka/config/panda_arm_hand.yaml')
    robot = Manipulator.loadFromURDF(urdf_path=panda_urdf_path, config_path=panda_config_path)
    print('robot id:{}, type:{}'.format(robot.id, type(robot.id)))
    
    # table, objects will be placed on top of it
    table = Box.fromParam(basePosition=[0, 0, 0.02],
                          baseRPY=[0, 0, 0],
                          size=[0.6, 1.2, 0.04],
                          useFixedBase=True)

    # cylinder to place the robot
    cylinder = Cylinder(basePosition=[-0.42, 0, 0.02], baseRPY=[0, 0, 0],
                        radius=0.1, length=0.04, rgbaColor=[0.3, 0.3, 0.3, 1],
                        useFixedBase=True)

    # load scene
    folder = os.path.join(rospack.get_path('ocrtoc_materials'), 'scenes')
    task_index = rospy.get_param('~task_index')
    world_path = folder + '/' + task_index + '.world'
    print('world_path: ', world_path)
    scene_object_ids = pybullet.loadSDF(world_path)
    print('scene_object_ids:', scene_object_ids)

    robot.goHome(0.2)

    from gazebo_msgs.msg import ModelState
    from gazebo_ros.gazebo_interface import GetModelState
    get_model_state_server = rospy.Service('/get_model_state', GetModelState, handle_get_model_state)

    # read all model poses
    model_id_name_map = get_model_id_name_map()
    for model_id, model_name in model_id_name_map.items():
        model_pose = pybullet.getBasePositionAndOrientation(model_id)
        print(model_id, model_name, model_pose)

    # reset model poses
    # pybullet.resetBasePositionAndOrientation(model_id, position, quaternion)

    while True:
        clock_msg = Clock()
        clock_msg.clock = rospy.Time.from_sec(sim_env.sim_time)
        #sim_time_publisher.publish(clock_msg)

        sim_env.step()
