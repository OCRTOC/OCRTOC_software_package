# Baseline Solution Perception

## Components
- Objects 6D poses estimation.
- Grasp pose detection.

## 6D pose estimation

### Shape based method

- Capture point clouds from different views (6 in the solution).
- Reconstruct the scene using the camera extrinsics.
- Remove the desktop and outliers in the point cloud.
- Cluster point cloud into several parts.
- Try ICP regitration with each object model of each point cloud part with several template initial pose.
- Assign a pose for each object.

### Correspondence based method

- Capture image from different views (6 in the solution).
- render images of objects from different views.
- Find correspondence on each captured image and rendered image.
- Calculate the 6d pose using RANSAC-PnP.

## Grasp pose detection

- Capture point clouds from different views (6 in the solution).
- Reconstruct the scene using the camera extrinsics.
- Generate grasp poses using graspnet-baseline.

## Modules

- Controlling Robot: `ocrtoc_perception/arm_controller.py`
- Object pose estimation: `ocrtoc_perception/pose`
    - Shape based method: `ocrtoc_perception/pose/pose_6d.py`
    - Correspondence based method: `ocrtoc_perception/pose/pose_correspondence.py`
- Grasp pose detection: `ocrtoc_perception/graspnet`

## Configuration file
- Parameters: `ocrtoc_perception/config/perception_franka_in_real.yaml`
    - pose_method: `superglue` for correspondence based method and `icp` for shape based method.
    - debug: `True` will print and display some of the infomation.
- Scanning Points: `ocrtoc_perception/config/arm_poses_Franka_realsense.csv`
