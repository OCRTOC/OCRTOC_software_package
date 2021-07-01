## CameraInterfaces
Example in [test/test_camera_interface.py](test/test_camera_interface.py)

1. Import
```
from ocrtoc_common.camera_interface import CameraInterface
```

2. Subscribe topics [CameraInfo, Image, PointCloud2]
```
camera_manager.subscribe_topic("/kinect/color/image_raw", Image)
camera_manager.subscribe_topic("/kinect/depth/image_raw", Image)
camera_manager.subscribe_topic("/kinect/depth/points", PointCloud2)
camera_manager.subscribe_topic("/kinect/color/camera_info", CameraInfo)
```

3. Get information，ROS format and python numpy/dict format
```
# color and depth image
camera_manager.get_ros_image("/kinect/color/image_raw")
camera_manager.get_numpy_image_with_encoding("/kinect/color/image_raw") # returns numpy.ndarray, encoding
# point cloud
camera_manager.get_ros_points("/kinect/depth/points")
camera_manager.get_numpy_points("/kinect/depth/points")

# camera information
camera_manager.get_ros_camera_info("/kinect/color/camera_info")
camera_manager.get_dict_camera_info("/kinect/color/camera_info")
```

## TransformInterface
Example in [test/test_transform_interface.py](test/test_transform_interface.py)

1. Import
```
from ocrtoc_common.transform_interface import TransformInterface
```

2. Functions
```
transform_manager = TransformInterface()
```
```
# Retrieve 'world' to 'panda_link8' transformation, with geometry_msgs/TransformStamped format
ros_trans = transform_manager.lookup_ros_transform('world', 'panda_link8')
```
```
# Retrieve 'world' to 'panda_link8' transformation, with numpy format
numpy_trans = transform_manager.lookup_numpy_transform('world', 'panda_link8')
```
```
# transpose pose in 'panda_link8' frame to 'world' frame
panda_link8_pose = PoseStamped()
panda_link8_pose.header.frame_id = "panda_link8"
panda_link8_pose.pose.position.x = 1.0
panda_link8_pose.pose.orientation.w = 1.0
world_pose = transform_manager.do_transform_ros_posestamped(
    panda_link8_pose, ros_trans)
```

## ManipulatorInterface
Example in  [test/test_manipulator_interface.py](test/test_manipulator_interface.py)

1. Start Pybullet
```
roslaunch ocrtoc_task bringup_simulator_pybullet.launch task_index:=0-0
```

2. Start moveit
```
roslaunch panda_moveit_config ocrtoc.launch
```

3. start ManipulatorInterface (python2)
```
rosrun ocrtoc_common manipulator_inrface_node.py
```

4. Call ManipulatorInterface (both python2 python3, communicate through rosservice)
```
roscd ocrtoc_common/test
python2 test_manipulator_interface.py
python3 test_manipulator_interface.py
```

## GripperInterface
Example in  [test/test_gripper_interface.py](test/test_gripper_interface.py)

1. Start Pybullet
```
roslaunch ocrtoc_task bringup_simulator_pybullet.launch task_index:=0-0
```

4. Call GripperInterface (both python2 python3)
```
roscd ocrtoc_common/test
python2 test_gripper_interface.py
python3 test_gripper_interface.py
```