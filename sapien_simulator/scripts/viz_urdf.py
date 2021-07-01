import argparse

import numpy as np
import sapien.core as sapien
from sapien.utils import Viewer


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-u', '--urdf', action='store', type=str, required=True, help="Path to the URDF file.")
    parser.add_argument('-s', '--simulate', action='store_true', default=False,
                        help="Whether to physically simulate the urdf.")
    return parser.parse_args()


def visualize_articulation(urdf_file, simulate):
    # Setup
    engine = sapien.Engine()
    renderer = sapien.VulkanRenderer(offscreen_only=False)
    engine.set_renderer(renderer)
    config = sapien.SceneConfig()
    config.gravity = np.array([0, 0, 0])
    scene = engine.create_scene(config=config)
    scene.set_timestep(1 / 125)
    visual_material = renderer.create_material()
    visual_material.set_base_color(np.array([132, 131, 101, 255]) / 255)
    visual_material.set_roughness(0.8)
    scene.add_ground(-1, render_material=visual_material)

    # Lighting
    render_scene = scene.get_renderer_scene()
    render_scene.set_ambient_light(np.array([0.6, 0.6, 0.6]))
    render_scene.add_directional_light(np.array([1, -1, -1]), np.array([0.5, 0.5, 0.5]))
    render_scene.add_point_light(np.array([2, 2, 2]), np.array([1, 1, 1]))
    render_scene.add_point_light(np.array([2, -2, 2]), np.array([1, 1, 1]))
    render_scene.add_point_light(np.array([-2, 0, 2]), np.array([1, 1, 1]))

    # Viewer
    print("hhh")
    viewer = Viewer(renderer)
    print("hhh")
    viewer.set_scene(scene)
    viewer.set_camera_xyz(1, 0, 1)
    viewer.set_camera_rpy(0, -0.6, 3.14)
    viewer.toggle_axes(0)

    # Articulation
    loader = scene.create_urdf_loader()
    robot = loader.load(urdf_file)
    robot.set_qpos(np.zeros([robot.dof]))
    scene.step()

    while not viewer.closed:
        scene.update_render()
        viewer.render()
        if simulate:
            scene.step()


def main():
    args = parse_args()
    visualize_articulation(args.urdf, args.simulate)


if __name__ == '__main__':
    main()
