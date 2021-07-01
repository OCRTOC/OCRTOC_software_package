# FAQ

1. Error in perception module: `"error processing request: conv2d(): argument 'input' (position 1) must be Tensor, not NoneType"`
- Reason: To accelerate computing, the rendered images are cached. Sometimes the cache is not complete. You need to delete all the cache.
- Solution: Delete all files in `/root/ocrtoc_ws/src/ocrtoc_perception/src/ocrtoc_perception/pose/rendered_object_images` EXCEPT `views.yaml`.

2. Error in moveit: `"PluginlibFactory: The plugin for class 'rviz_visual_tools/RvizVisualToolsGui' failed to load."`  
- Solution: It doesn't matter.

3. Error in x server when debug is set to `True` while using `superglue` perception method.
- Reason: This problem is caused by pyrender. After rendering images, open3d and matplotlib visualization can not be used. 
- Solution: Currently, you need to delete all the visualization after rendering.

4. Why Sapien simulator stops for a short time each second.  
- Reason: Sapien renders and publishes images at a frequency of 1 Hz. As a result, when the graphics engine is rendering, the physics engine waits until it ends. However, this doesn't affect the phisical process. 

5. Error in Sapien simulator: `"A controller named 'position_joint_trajectory_controller' was already loaded inside the controller manager"`  
- Solution: It doesn't matter, you can continue using the simulator. 

6. Can I read the object 6d poses from simulators?
- Answer: You may use the `/get_model_state` interface while debugging. An example is in [`ocrtoc_task/scripts/get_6dpose_dump_simulator.py`](ocrtoc_task/scripts/get_6dpose_dump_simulator.py). But you should NEVER do that in your solution. The scores will be cancelled if we find that you read the poses directly from the simulator.
