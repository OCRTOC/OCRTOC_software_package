# OCRTOC-PyBullet
PyBullet simulator support for OCRTOC (Open Cloud Robot Table Organization Challenge).

System Environment
-------------------
Ubuntu 18.04
Python 2.7

PyBullet Installation
-------------------

* To be compatible with ROS melodic on Ubuntu 18.04, we will use python2 since it is the supported version for ROS melodic.

```sh
pip install pybullet
```
(you may need 'sudo' to install, if you got error installing pybullet, try to upgrade numpy to newest version with 'pip install -U numpy')



Run Pybullet
-------------------
Open a terminal, load the simulation environment with:
```sh
roslaunch ocrtoc_task bringup_simulator_pybullet.launch task_index:=0-0
```

Run Pybullet ros interface:
```sh
rosrun pybullet_simulator ros_interface.py
```

Moveit
-------------------
Open a terminal:
```sh
roslaunch panda_moveit_config ocrtoc.launch
```

Perception
-------------------
Open a terminal:
```sh
roslaunch ocrtoc_perception perception_solution.launch
```

Planning
-------------------
Open a terminal:
```sh
roslaunch ocrtoc_planning planning_solution.launch
```

Trigger
-------------------
Open a terminal:
```sh
roslaunch ocrtoc_task trigger.launch task_index:=0-0
```