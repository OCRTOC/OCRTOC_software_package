#! /usr/bin/python3
import os
import sys
import time
import rospy

import numpy as np
import sapien.core as sapien
import sapien.ros1 as sr
from sapien.utils import Viewer

sys.path.append(os.path.abspath(os.path.dirname(__file__)))
from asset_utils import load_sapien_sdf, load_goal_sdf
import robot_const

SPEED_UP = 1
RENDER_HZ = 8 * SPEED_UP


def main():
    materials_dir = rospy.get_param('~materials_dir')
    gui = rospy.get_param('~gui')
    robot_name = rospy.get_param('~robot')
    world_path = rospy.get_param('~world_path')

    os.environ["SAPIEN_MODEL_PATH"] = os.path.join(materials_dir, "models")

    # robot name
    if robot_name not in robot_const.SUPPORT_ROBOT:
        raise ValueError(
            f"Robot name {robot_name} is not supported. Currently we only support {robot_const.SUPPORT_ROBOT}")

    engine = sapien.Engine()
    engine.set_log_level("error")
    renderer = sapien.VulkanRenderer(offscreen_only=not gui)
    renderer.set_log_level("error")
    engine.set_renderer(renderer)

    # Load scene
    scene_config = sapien.SceneConfig()
    scene_config.solver_iterations = 20
    scene_config.solver_velocity_iterations = 1
    scene_config.enable_pcm = False
    scene_config.default_restitution = 0
    scene_config.default_dynamic_friction = 0.5
    scene_config.default_static_friction = 0.8
    scene = engine.create_scene(scene_config)
    scene.set_timestep(1 / 250)

    # Load ground
    ground_material = renderer.create_material()
    ground_color = np.array([202, 164, 114, 256]) / 256
    ground_material.set_base_color(ground_color)
    ground_material.set_specular(0.5)
    scene.add_ground(-0.8, render_material=ground_material)

    # Load sdf
    table_height = 0.0
    sdf_objects, id2name, id2scale = load_sapien_sdf(world_path, scene, table_height)
    sdf_goal = load_goal_sdf(world_path.replace("scenes", "targets"), scene, table_height)

    # Render scene
    render_scene = scene.get_renderer_scene()
    render_scene.set_ambient_light(np.array([0.4, 0.4, 0.4]))

    # Configure robot
    scene_manager = sr.SceneManager(scene, "")
    loader = scene_manager.create_robot_loader()
    loader.fix_root_link = True
    gripper_material = engine.create_physical_material(3.0, 2.8, 0.01)
    urdf_config = {"link": dict()}
    for gripper_name in robot_const.GRIPPER_NAMES[robot_name]:
        urdf_config["link"].update(
            {gripper_name: {"material": gripper_material, "patch_radius": 0.02, "min_patch_radius": 0.005}})

    # Load robot
    robot, manager = loader.load_from_parameter_server("", urdf_config, 125,
                                                       mimic_joint_names=robot_const.MIMIC_JOINT_NAMES[robot_name])
    init_qpos = robot_const.INIT_QPOS[robot_name]
    robot.set_qpos(init_qpos)
    robot.set_drive_target(init_qpos)
    scene.step()
    scene_manager.start_get_model_service("/get_model_state", sdf_objects)
    scene_manager.start_remove_model_service("/remove_model")
    for cam, actor in zip(scene.get_mounted_cameras(), scene.get_mounted_actors()):
        scene_manager.start_all_ros_camera(cam, actor, 1, cloud_cutoff_min=0.25, cloud_cutoff_max=5,
                                           publish_point_cloud=True)

    # Start
    scene_manager.start()
    step = 0
    timestep = scene.get_timestep() / SPEED_UP
    next_step_time = time.time() + timestep
    for drive_property in robot_const.DRIVE_PROPERTY[robot_name]:
        manager.set_drive_property(*drive_property)

    # Viewer
    if gui:
        viewer = Viewer(renderer)
        viewer.set_scene(scene)
        viewer.set_camera_xyz(1.5, 0, 1.5)
        viewer.set_camera_rpy(0, -0.7, 3.14)

        def show_goal():
            for sdf in sdf_goal:
                current_pose = sdf.get_pose()
                position = current_pose.p
                position[2] += 100
                sdf.set_pose(sapien.Pose(position, current_pose.q))

        def hide_goal():
            for sdf in sdf_goal:
                current_pose = sdf.get_pose()
                position = current_pose.p
                position[2] = position[2] % 100
                sdf.set_pose(sapien.Pose(position, current_pose.q))

        viewer.key_press_action_map.update({"z": show_goal, "x": hide_goal})

    # execution_time = 0
    if gui:
        while not viewer.closed:
            step_and_render(manager, scene, viewer, step, next_step_time)
            next_step_time += timestep
            step += 1
            # execution_time += is_execution_now(robot) * scene.get_timestep()
    else:
        try:
            while True:
                step_only(manager, scene, next_step_time)
                next_step_time += timestep
                step += 1
                # execution_time += is_execution_now(robot) * scene.get_timestep()
        except KeyboardInterrupt:
            print("Simulation stopped by user")

    scene = None


def step_and_render(manager, scene, viewer, step, next_step_time):
    manager.balance_passive_force()
    now = time.time()
    while now < next_step_time:
        time.sleep(1e-4)
        now = time.time()
    scene.step()
    scene.update_render()
    if step % RENDER_HZ == 0:
        viewer.render()


def step_only(manager, scene, next_step_time):
    manager.balance_passive_force()
    now = time.time()
    while now < next_step_time:
        time.sleep(1e-4)
        now = time.time()
    scene.step()
    scene.update_render()


def is_execution_now(robot):
    qvel = np.abs(robot.get_qvel()[:8])
    qacc = np.abs(robot.get_qacc()[:8])
    return int(np.min(qvel) > 1e-2 or np.min(qacc) > 1e-2)


if __name__ == '__main__':
    rospy.init_node('sapien_ros')

    sr.ros_init("ocrtoc_pipeline", sys.argv)

    main()
