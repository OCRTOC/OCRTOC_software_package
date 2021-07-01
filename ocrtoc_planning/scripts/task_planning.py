from __future__ import print_function
import argparse
import copy
import math
import numpy as np
import transforms3d
import os
import yaml

from std_msgs.msg import String, Bool, Empty, Int64
from gazebo_msgs.srv import *
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from motion_planning import MotionPlanner
from ocrtoc_common.transform_interface import TransformInterface
from ocrtoc_msg.srv import PerceptionTarget, PerceptionTargetRequest
import rospkg
import rospy

sys.path.append(os.path.join(os.path.dirname(__file__), 'pddlstream'))
from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.language.constants import And, Equal, TOTAL_COST, print_solution
from pddlstream.language.generator import from_gen_fn, from_fn, from_test
from pddlstream.language.stream import StreamInfo
from pddlstream.language.generator import outputs_from_boolean
from pddlstream.utils import user_input, read, INF
from primitives import GRASP, collision_test, distance_fn, DiscreteTAMPState, get_shift_one_problem, get_shift_all_problem
from viewer import DiscreteTAMPViewer, COLORS


class TaskPlanner(object):
    """Plan object operation sequence for the whole task.

    Receive the names and target poses of target objects. Get the current poses of target objects by calling the
    perception node. Then plan the sequence operate the list of objects. Finally, complete each object operation
    with the class called MotionPlanner.
    """

    def __init__(self, blocks=[], goal_cartesian_poses=[]):
        """Inits TaskPlanner with object names and corresponding target poses.

        :param blocks: A list of object names
        :param goal_cartesian_poses: A list of target poses of objects
        """
        print("start construct task planner")
        # load parameters
        rospack = rospkg.RosPack()
        task_config_path = os.path.join(rospack.get_path('ocrtoc_planning'), 'config/task_planner_parameter.yaml')
        with open(task_config_path, "r") as f:
            config_parameters = yaml.load(f)
            self._max_repeat_call = config_parameters["max_repeat_call"]
            self._grasp_distance = config_parameters["grasp_distance"]
            self._min_angle = config_parameters["min_angle"] * math.pi / 180
            self._grasp_up_translation = config_parameters["grasp_up_translation"]
            self._start_grasp_index = config_parameters["start_grasp_index"]
            self._end_grasp_index = config_parameters["end_grasp_index"]
            self._pick_via_up = config_parameters["pick_via_up"]
            # if the plane distance between two objects smaller than distance threshold,
            # the two objects are considered to be stacked
            self._distance_threshold = config_parameters["distance_threshold"]
        self._task_planner_sub = rospy.Subscriber('/start_task_planner', Int64, self.task_planner_callback, queue_size=1)
        self._service_get_sim_pose = rospy.ServiceProxy('/get_model_state', GetModelState)
        self._service_get_target_pose = rospy.ServiceProxy('/perception_action_target', PerceptionTarget)
        self._service_get_fake_pose = rospy.ServiceProxy('/fake_perception_action_target', PerceptionTarget)
        self._algorithm_arc = 'focused'  # 'incremental'
        self._world_axis_z = np.array([0, 0, 1])
        self._unit_arc = False
        self._transformer = TransformInterface()
        self._motion_planner = MotionPlanner()
        self._n_repeat_call = 0
        self._blocks = blocks
        self._initial_cartesian_poses = []
        self._current_block_poses = {}
        self.simplify_task(goal_cartesian_poses)
        self._n_blocks = len(blocks)
        self._temp_block_poses = []
        self._temp_cartesian_poses = []
        self._available_block_pose_dic = {}
        self._available_cartesian_pose_dic = {}
        self._available_grasp_pose_dic = {}
        self._goal_block_pose_dic = {}
        self._target_cartesian_pose_dic = {}
        self._target_block_pose_dic = {}
        self._pose_mapping = {}
        self._target_grasp_pose_dic = {}
        self._goal_cartesian_pose_dic = dict(zip(self._blocks, self._goal_cartesian_poses))
        self._available_grasp_pose_index = {}
        self._available_block_pose_inverse = {}
        self._n_available_grasp_pose = 1
        self._last_gripper_action = 'place'
        self._target_pick_object = None
        print("number of blocks: {}".format(self._n_blocks))
        print("blocks: {}".format(self._blocks))
        print("goal cartesian poses dictionary: {}".format(self._goal_cartesian_pose_dic))
        print("task planner constructed")

    def simplify_task(self, goal_cartesian_poses):
        self._goal_cartesian_poses = []
        for i in goal_cartesian_poses:
            self._goal_cartesian_poses.append(i.poses[0])

    def get_pose_perception(self, target_object_list):
        """Get current poses of target objects from perception node.

        :param target_object_list: A list of target object names
        """

        # perception service message:
        # string[] target_object_list
        # ---
        # ocrtoc_perception/PerceptionResult[] perception_result_list
        # ocrtoc_perception/PerceptionResult.msg
        # string object_name
        # bool be_recognized
        # geometry_msgs/PoseStamped object_pose
        # bool is_graspable
        # geometry_msgs/PoseStamped grasp_pose
        rospy.wait_for_service('/perception_action_target')
        request_msg = PerceptionTargetRequest()
        request_msg.target_object_list = target_object_list
        rospy.loginfo('Start to call perception node')
        perception_result = self._service_get_target_pose(request_msg)
        rospy.loginfo('Perception finished')

        self._available_cartesian_pose_dic.clear()
        self._available_grasp_pose_dic.clear()
        self._available_grasp_pose_index.clear()
        rospy.loginfo(str(len(perception_result.perception_result_list)) + ' objects are graspable:')
        rospy.loginfo('Graspable objects information: ')

        for result in perception_result.perception_result_list:
            if result.be_recognized and result.is_graspable:
                self._available_cartesian_pose_dic[result.object_name] = result.object_pose.pose
                self._available_grasp_pose_dic[result.object_name] = []  # pick pose list

                # intelligence grasp strategy
                self._available_grasp_pose_dic[result.object_name].append(result.grasp_pose.pose)  # intelligence pick pose

                # artificial intelligence grasp strategy
                artificial_intelligence_grasp_pose = self.get_artificial_intelligence_grasp_pose(result.grasp_pose.pose)
                self._available_grasp_pose_dic[result.object_name].append(artificial_intelligence_grasp_pose)  # artificial intelligence pick pose

                # artificial grasp strategy
                artificial_grasp_pose = Pose()
                artificial_grasp_pose.position.x = result.object_pose.pose.position.x
                artificial_grasp_pose.position.y = result.object_pose.pose.position.y
                artificial_grasp_pose.position.z = result.object_pose.pose.position.z + self._grasp_distance
                artificial_grasp_pose.orientation.x = 0
                artificial_grasp_pose.orientation.y = 1
                artificial_grasp_pose.orientation.z = 0
                artificial_grasp_pose.orientation.w = 0
                self._available_grasp_pose_dic[result.object_name].append(artificial_grasp_pose)  # artificial pick pose

                self._target_grasp_pose_dic[result.object_name] = []  # place pose list

                # self.get_target_grasp_pose(result.object_pose.pose, result.grasp_pose.pose, self._goal_cartesian_pose_dic[result.object_name])
                target_grasp_intelligence = \
                    self.get_target_grasp_pose2(result.object_pose.pose, result.grasp_pose.pose, self._goal_cartesian_pose_dic[result.object_name])
                self._target_grasp_pose_dic[result.object_name].append(target_grasp_intelligence)  # intelligence place pose

                # self.get_target_grasp_pose(result.object_pose.pose, artificial_intelligence_grasp_pose, self._goal_cartesian_pose_dic[result.object_name])
                target_grasp_pose_artificial_intelligence = \
                    self.get_target_grasp_pose2(result.object_pose.pose, artificial_intelligence_grasp_pose, self._goal_cartesian_pose_dic[result.object_name])
                self._target_grasp_pose_dic[result.object_name].append(target_grasp_pose_artificial_intelligence)  # artificial intelligence place pose

                # self.get_target_grasp_pose(result.object_pose.pose, artificial_grasp_pose, self._goal_cartesian_pose_dic[result.object_name])
                target_grasp_artificial = \
                    self.get_target_grasp_pose2(result.object_pose.pose, artificial_grasp_pose, self._goal_cartesian_pose_dic[result.object_name])
                self._target_grasp_pose_dic[result.object_name].append(target_grasp_artificial)  # artificial place pose

                self._available_grasp_pose_index[result.object_name] = self._start_grasp_index if self._start_grasp_index >= 0 else 0
                print('object name: {0}, frame id: {1}, cartesian pose: {2}'.format(result.object_name, result.object_pose.header.frame_id, result.object_pose.pose))
                print('object name: {0}, frame id: {1}, grasp pose: {2}'.format(result.object_name, result.grasp_pose.header.frame_id, self._available_grasp_pose_dic[result.object_name]))
            else:
                pass

    # get pose from simulation environment
    def get_fake_pose_perception(self, target_object_list):
        rospy.wait_for_service('/fake_perception_action_target')
        request_msg = PerceptionTargetRequest()
        request_msg.target_object_list = target_object_list
        perception_result = self._service_get_fake_pose(request_msg)
        self._available_cartesian_pose_dic.clear()
        self._available_grasp_pose_dic.clear()
        for result in perception_result.perception_result_list:
            if result.be_recognized and result.is_graspable:
                self._available_cartesian_pose_dic[result.object_name] = result.object_pose.pose
                self._available_grasp_pose_dic[result.object_name] = result.grasp_pose.pose
                self._target_grasp_pose_dic[result.object_name] = \
                self.get_target_grasp_pose(result.object_pose.pose, result.grasp_pose.pose, self._goal_cartesian_pose_dic[result.object_name])
            else:
                pass
        rospy.loginfo('fake pose obtained')
        print('available objects and cartesian pose: {}'.format(self._available_cartesian_pose_dic))
        print('available objects and grasp pose: {}'.format(self._available_grasp_pose_dic))
        print('available objects target grasp pose: {}'.format(self._target_grasp_pose_dic))

    # modulate intelligence grasp pose to avoid collision
    def get_artificial_intelligence_grasp_pose(self, intelligence_grasp_pose):
        artificial_intelligence_grasp_pose = Pose()
        intelligence_pose_matrix = self._transformer.ros_pose_to_matrix4x4(intelligence_grasp_pose)
        intelligence_pose_axis_z = intelligence_pose_matrix[0:3, 2]
        print('intelligence pose z axis: {}'.format(intelligence_pose_axis_z))
        intelligence_cos_alpha = np.dot(intelligence_pose_axis_z, self._world_axis_z)
        intelligence_alpha = math.acos(intelligence_cos_alpha)
        print('intelligence pose angle (degree): {}'.format(intelligence_alpha * 180 / math.pi))

        if intelligence_alpha > self._min_angle:
            rospy.loginfo('intelligence pose in specified range, no need to adjust')
            artificial_intelligence_grasp_pose = intelligence_grasp_pose
        else:
            rospy.loginfo('intelligence pose out of specified range, may cause undesired behavior, it should be adjusted')
            delta_angle = self._min_angle - intelligence_alpha
            rotation_axis = np.cross(intelligence_pose_axis_z, -self._world_axis_z)
            rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
            print('rotation axis in world frame:\n{}'.format(rotation_axis))
            intelligence_rotation_matrix = intelligence_pose_matrix[0:3, 0:3]
            rotation_axis = np.matmul(np.linalg.inv(intelligence_rotation_matrix), rotation_axis)  # transform rotation axis from world frame to end-effector frame
            print('rotation axis in end frame: \n{}'.format(rotation_axis))
            rotation_quaternion = np.array([math.cos(delta_angle/2), rotation_axis[0]*math.sin(delta_angle/2),\
            rotation_axis[1]*math.sin(delta_angle/2), rotation_axis[2]*math.sin(delta_angle/2)])  # [w, x, y, z]
            rotation_matrix = transforms3d.quaternions.quat2mat(rotation_quaternion)
            transformation_matrix = np.eye(4, dtype=np.float)
            transformation_matrix[0:3, 0:3] = rotation_matrix
            artificial_intelligence_grasp_pose_matrix = np.matmul(intelligence_pose_matrix, transformation_matrix)  # do rotation

            linear_transformation_matrix = np.eye(4, dtype=np.float)
            linear_transformation_matrix[2, 3] = self._grasp_up_translation
            artificial_intelligence_grasp_pose_matrix = np.matmul(linear_transformation_matrix, artificial_intelligence_grasp_pose_matrix)  # do upright translation
            artificial_intelligence_grasp_pose = self._transformer.matrix4x4_to_ros_pose(artificial_intelligence_grasp_pose_matrix)

            artificial_intelligence_grasp_pose_axis_z = artificial_intelligence_grasp_pose_matrix[0:3, 2]
            artificial_intelligence_cos_alpha = np.dot(artificial_intelligence_grasp_pose_axis_z, self._world_axis_z)
            artificial_intelligence_alpha = math.acos(artificial_intelligence_cos_alpha)
            print('delta angle: {}'.format(delta_angle * 180 / math.pi))
            print('artificial intelligence pose angle (degree): {}'.format(artificial_intelligence_alpha * 180 / math.pi))
            print('artificial intelligence pose z axis: {}'.format(artificial_intelligence_grasp_pose_axis_z))

        return artificial_intelligence_grasp_pose

    # get grasp pose in target configuration
    def get_target_grasp_pose(self, pose1, grasp_pose1, pose2):
        pose1_matrix = self._transformer.ros_pose_to_matrix4x4(pose1)
        grasp_pose1_matrix = self._transformer.ros_pose_to_matrix4x4(grasp_pose1)
        transformation_matrix = np.dot(np.linalg.inv(pose1_matrix), grasp_pose1_matrix)

        pose2_matrix = self._transformer.ros_pose_to_matrix4x4(pose2)
        grasp_pose2_matrix = np.dot(pose2_matrix, transformation_matrix)
        grasp_pose2_matrix[2, 3] = grasp_pose2_matrix[2, 3] + 0.02  # to avoid collision with desk
        grasp_pose2 = self._transformer.matrix4x4_to_ros_pose(grasp_pose2_matrix)

        return grasp_pose2

    # get grasp pose in target configuration
    def get_target_grasp_pose2(self, pose1, grasp_pose1, pose2):
        pose1_matrix = self._transformer.ros_pose_to_matrix4x4(pose1)
        grasp_pose1_matrix = self._transformer.ros_pose_to_matrix4x4(grasp_pose1)
        transformation_matrix = np.dot(np.linalg.inv(pose1_matrix), grasp_pose1_matrix)

        # change target pose orientation to initial pose orientation
        pose2.orientation.x = pose1.orientation.x
        pose2.orientation.y = pose1.orientation.y
        pose2.orientation.z = pose1.orientation.z
        pose2.orientation.w = pose1.orientation.w

        pose2_matrix = self._transformer.ros_pose_to_matrix4x4(pose2)
        grasp_pose2_matrix = np.dot(pose2_matrix, transformation_matrix)
        grasp_pose2_matrix[2, 3] = grasp_pose2_matrix[2, 3] + 0.02  # to avoid collision with desk
        grasp_pose2 = self._transformer.matrix4x4_to_ros_pose(grasp_pose2_matrix)

        return grasp_pose2

    # judge whether two objects are stacked
    def is_stacked(self, pose1, pose2):
        plane_distance = math.sqrt((pose1.position.x - pose2.position.x)**2 + (pose1.position.y - pose2.position.y)**2)
        return True if plane_distance < self._distance_threshold else False

    # construct pose transformation of initial objects
    def available_pose_transformation(self):
        sorted_available_cartesian_pose_list = sorted(self._available_cartesian_pose_dic.items(), key=lambda obj: obj[1].position.z)
        print("sorted available cartesian pose list element: {}".format(sorted_available_cartesian_pose_list[0]))
        print("sub element1: {0}, sub element2: {1}".format(sorted_available_cartesian_pose_list[0][0], sorted_available_cartesian_pose_list[0][1]))
        blocks_x = 0
        self._available_block_pose_dic.clear()
        for i in range(len(sorted_available_cartesian_pose_list)):
            if i == 0:
                self._available_block_pose_dic[sorted_available_cartesian_pose_list[0][0]] = np.array([0, 0])
                blocks_x += 1
            else:
                for j in range(i):
                    if self.is_stacked(sorted_available_cartesian_pose_list[j][1], sorted_available_cartesian_pose_list[i][1]):
                        x, y = self._available_block_pose_dic[sorted_available_cartesian_pose_list[j][0]]
                        y += 1
                        self._available_block_pose_dic[sorted_available_cartesian_pose_list[i][0]] = np.array([x, y])
                    elif j == (i - 1):
                        self._available_block_pose_dic[sorted_available_cartesian_pose_list[i][0]] = np.array([blocks_x, 0])
                        blocks_x += 1
                    else:
                        pass
        print("available block pose dictionary: {}".format(self._available_block_pose_dic))

        # generate object/pose inverse mapping
        for block, pose in self._available_block_pose_dic.items():
            self._available_block_pose_inverse[str(pose)] = block

        # map the object pose in block space to cartesian space
        for block in self._available_cartesian_pose_dic.keys():
            self._pose_mapping[str(self._available_block_pose_dic[block])] = self._available_grasp_pose_dic[block]
        print("available grasp pose mapping: {}".format(self._pose_mapping))

    # construct pose transformation of all objects for goal state (and temperary pose), call only once
    def goal_pose_transformation(self):
        blocks_x = self._n_blocks
        # construct 2d pose for target object cartesian pose
        sorted_goal_cartesian_pose_list = sorted(self._goal_cartesian_pose_dic.items(), key=lambda obj: obj[1].position.z)
        for i in range(len(sorted_goal_cartesian_pose_list)):
            if i == 0:
                self._goal_block_pose_dic[sorted_goal_cartesian_pose_list[0][0]] = np.array([blocks_x, 0])
                blocks_x += 1
            else:
                for j in range(i):
                    if self.is_stacked(sorted_goal_cartesian_pose_list[j][1], sorted_goal_cartesian_pose_list[i][1]):
                        x, y = self._goal_block_pose_dic[sorted_goal_cartesian_pose_list[j][0]]
                        y += 1
                        self._goal_block_pose_dic[sorted_goal_cartesian_pose_list[i][0]] = np.array([x, y])
                    elif j == (i - 1):
                        self._goal_block_pose_dic[sorted_goal_cartesian_pose_list[i][0]] = np.array([blocks_x, 0])
                        blocks_x += 1
                    else:
                        pass
        print("goal block pose dictionary: {}".format(self._goal_block_pose_dic))

        # construct 2d pose temporary cartesian pose
        self._temp_block_poses = [np.array([blocks_x + x, 0]) for x in range(len(self._temp_cartesian_poses))]
        str_temp_pose = []
        for temp_pose in self._temp_block_poses:
            str_temp_pose.append(str(temp_pose))
        if len(str_temp_pose) > 0:
            self._pose_mapping.update(dict(zip(str_temp_pose, self._temp_cartesian_poses)))

    # map the object pose in block space to grasp space
    def target_grasp_pose_transformation(self):
        for block in self._available_grasp_pose_dic.keys():
            self._pose_mapping[str(self._goal_block_pose_dic[block])] = self._target_grasp_pose_dic[block]
        print('whole pose mapping:{}'.format(self._pose_mapping))

    # get poses of part of objects each call of perception node, call perception node and plan task several times
    def cycle_plan_all(self):
        """Plan object operation sequence and execute operations recurrently.

        logical process:
        1 get all objects goal poses
        2 get current objects poses
        # 3 compare current objects poses and goal poses
        4 plan task sequence of current relative objects
        5 task execution
        6 delete already planned objects from goal objects list and check goal objects list is empty
        7 if goal objects list is empty, stop planning, otherwise, go to step 2
        """

        print("enter cycle_plan_all function")
        # transform goal cartesian pose to 2d space
        self.goal_pose_transformation()

        # TODO check task result???
        left_object_dic = self._goal_block_pose_dic
        while len(left_object_dic) != 0:
            # get left objects information
            rospy.loginfo('Try to get information of left objects from perception node')
            self.get_pose_perception(left_object_dic.keys())  # get pose from perception
            # self.get_fake_pose_perception(left_object_dic.keys())  # get pose from simulation environment

            # if no availabe object information get, call perception node again until the number of repeat call reach max_repeat_call
            if len(self._available_cartesian_pose_dic) == 0:
                rospy.loginfo('Nothing get from perception, but the information of the following objects have never been gotten. Call perpectp again 3 seconds later')
                print('left objects: {}'.format(left_object_dic.keys()))
                rospy.sleep(3.0)
                self._n_repeat_call += 1
                if self._n_repeat_call >= self._max_repeat_call:
                    rospy.loginfo('Call perception node' + str(self._max_repeat_call) + 'times, but nothing get')
                    rospy.loginfo('Task failed, Information of the following objects can not be obtained: ')
                    print(left_object_dic.keys())
                    break
                else:
                    continue

            # compare goal objects and currently available objects
            self._target_cartesian_pose_dic.clear()
            self._target_block_pose_dic.clear()
            for block in self._blocks:
                if block in self._available_cartesian_pose_dic.keys():
                    self._target_cartesian_pose_dic[block] = self._goal_cartesian_pose_dic[block]
                    self._target_block_pose_dic[block] = self._goal_block_pose_dic[block]
                else:
                    pass

            # transform available cartesian pose to 2d space
            self.available_pose_transformation()

            # get target grasp pose to 2d space
            self.target_grasp_pose_transformation()

            # task planning and execution
            self.target_objects_task_planning()

            # delete completed task, update left objects list
            for block in self._target_cartesian_pose_dic.keys():
                del left_object_dic[block]

            print('left objects: {}'.format(left_object_dic))

    # task planning and execution of objects that are currently available and in the list of goal objects
    def target_objects_task_planning(self):
        problem_fn = get_shift_all_problem
        task_planning_problem = problem_fn(self._available_block_pose_dic, self._target_block_pose_dic, self._temp_block_poses)
        print('====================task planning problem:====================')
        print(task_planning_problem)
        print('==============================================================')

        stream_info = {
            'test-cfree': StreamInfo(negate=True),
        }

        pddlstream_problem = self.pddlstream_from_tamp(task_planning_problem)
        print('==========================algorithm===========================')
        if self._algorithm_arc == 'focused':
            solution = solve_focused(pddlstream_problem, unit_costs=self._unit_arc, stream_info=stream_info, debug=False)
        elif self._algorithm_arc == 'incremental':
            solution = solve_incremental(pddlstream_problem, unit_costs=self._unit_arc,
                                         complexity_step=INF)
        else:
            raise ValueError(self._algorithm_arc)
        print('==============================================================')

        print('========================printSolution=========================')
        print_solution(solution)
        print('==============================================================')
        plan, cost, evaluations = solution
        if plan is None:
            return
        print('=========================apply_plan===========================')
        self.apply_plan(task_planning_problem, plan)
        print('==============================================================')

    def once_plan_all(self):
        print("enter once_plan_all function")

        # get all pose information in simulation environment
        self.get_pose_perception(self._blocks)

        # transpose initial and target pose of all objects to 2d space
        self.goal_pose_transformation()
        self.available_pose_transformation()

        problem_fn = get_shift_all_problem  # get_shift_one_problem | get_shift_all_problem
        tamp_problem = problem_fn(self._available_block_pose_dic, self._goal_block_pose_dic, self._temp_block_poses)
        print('====================tamp_problem:====================')
        print(tamp_problem)
        print('=====================================================')

        stream_info = {
            'test-cfree': StreamInfo(negate=True),
        }

        pddlstream_problem = self.pddlstream_from_tamp(tamp_problem)
        print('====================algorithm=====================')
        if self._algorithm_arc == 'focused':
            solution = solve_focused(pddlstream_problem, unit_costs=self._unit_arc, stream_info=stream_info, debug=False)
        elif self._algorithm_arc == 'incremental':
            solution = solve_incremental(pddlstream_problem, unit_costs=self._unit_arc,
                                         complexity_step=INF)  # max_complexity=0)
        else:
            raise ValueError(self._algorithm_arc)
        print('==================================================')

        print('====================printSolution=====================')
        print_solution(solution)
        print('======================================================')
        plan, cost, evaluations = solution
        if plan is None:
            return
        print('====================apply_plan=====================')
        self.apply_plan(tamp_problem, plan)
        print('===================================================')

    def pddlstream_from_tamp(self, tamp_problem):
        initial = tamp_problem.initial
        assert (initial.holding is None)

        known_poses = list(initial.block_poses.values()) + list(tamp_problem.goal_poses.values())

        directory = os.path.dirname(os.path.abspath(__file__))
        domain_pddl = read(os.path.join(directory, 'domain.pddl'))
        stream_pddl = read(os.path.join(directory, 'stream.pddl'))

        q100 = np.array([100, 100])
        constant_map = {
            'q100': q100,  # As an example
        }

        init = [
                   ('CanMove',),
                   ('Conf', q100),
                   ('Conf', initial.conf),
                   ('AtConf', initial.conf),
                   ('HandEmpty',),
                   Equal((TOTAL_COST,), 0)] + \
               [('Block', b) for b in initial.block_poses.keys()] + \
               [('Pose', p) for p in known_poses] + \
               [('AtPose', b, p) for b, p in initial.block_poses.items()]
        # [('Pose', p) for p in known_poses + tamp_problem.poses] + \

        goal = And(*[
            ('AtPose', b, p) for b, p in tamp_problem.goal_poses.items()
        ])

        # TODO: convert to lower case
        stream_map = {
            # 'sample-pose': from_gen_fn(lambda: ((np.array([x, 0]),) for x in range(len(poses), n_poses))),
            'sample-pose': from_gen_fn(lambda: ((p,) for p in tamp_problem.poses)),
            'inverse-kinematics': from_fn(lambda p: (p + GRASP,)),
            'test-cfree': from_test(lambda *args: not collision_test(*args)),
            'collision': collision_test,
            'distance': distance_fn,
            'test-ontop': self.is_ontop(lambda *args: self.ontop_test(*args)),
        }

        return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

    def ontop_test(self, *args):
        # print("ontop_test args: {}".format(args))
        # x, y = args
        # if y < 0:
        #     return True  # to handle the initial configuration of gripper [0, -1] in 2d space
        # y += 1
        # up = np.array([x, y])
        return True

    def is_ontop(self, test):
        print('on top function')
        return from_fn(lambda *args, **kwargs: outputs_from_boolean(test(*args, **kwargs)))

    def draw_state(self, viewer, state, colors):
        print("enter draw state function")
        viewer.clear()
        viewer.draw_environment()
        viewer.draw_robot(*state.conf[::-1])
        for block, pose in state.block_poses.items():
            r, c = pose[::-1]
            viewer.draw_block(r, c, name=block, color=colors[block])
        if state.holding is not None:
            pose = state.conf - GRASP
            r, c = pose[::-1]
            viewer.draw_block(r, c, name=state.holding, color=colors[state.holding])

    # mapping move action in 2d space to cartesian space
    def mapping_move(self, str_pose):
        cartesian_waypoints = []
        if str_pose in self._pose_mapping.keys():
            if self._last_gripper_action == 'pick':
                grasp_pose = self._pose_mapping[str_pose][self._available_grasp_pose_index[self._target_pick_object]]
                # plan_result = self._motion_planner.move_cartesian_space(grasp_pose)  # move in cartesian straight path
                # plan_result = self._motion_planner.move_cartesian_space_discrete(grasp_pose)  # move in cartesian discrete path
                plan_result = self._motion_planner.move_cartesian_space_upright(grasp_pose)  # move in cartesian discrete upright path
                if plan_result:
                    print('Move to the target position of object {} successfully, going to place it'.format(self._target_pick_object))
                else:
                    print('Fail to move to the target position of object: {}'.format(self._target_pick_object))
                    #user_input('To target place failed, Continue? Press Enter to continue or ctrl+c to stop')
                rospy.sleep(1.0)
            elif self._last_gripper_action == 'place':
                for index in range(self._start_grasp_index if self._start_grasp_index >= 0 else 0, self._end_grasp_index if self._end_grasp_index <= len(self._pose_mapping[str_pose]) else len(self._pose_mapping[str_pose])):
                    # plan_result = self._motion_planner.move_cartesian_space(self._pose_mapping[str_pose][index], self._pick_via_up)
                    # plan_result = self._motion_planner.move_cartesian_space_discrete(self._pose_mapping[str_pose][index], self._pick_via_up)
                    plan_result = self._motion_planner.move_cartesian_space_upright(self._pose_mapping[str_pose][index], self._pick_via_up)
                    if plan_result:
                        self._available_grasp_pose_index[self._target_pick_object] = index
                        rospy.loginfo('available grasp pose index: ' + str(index))
                        print('target pick object: {}'.format(self._target_pick_object))
                        if index == 0:
                            rospy.loginfo('Planning trajectory to intelligence grasp pose successfully')
                        elif index == 1:
                            rospy.loginfo('Fail to plan trajectory to intelligence grasp pose, but planning trajectory to artifical intelligence grasp pose successfully')
                        elif index == 2:
                            rospy.loginfo('Fail to plan trajectory to artificial intelligence grasp pose, but planning trajectory to artifical grasp pose successfully')
                        else:
                            pass
                        break
                    else:
                        #user_input('To pick action failed, Continue? Press Enter to continue or ctrl+c to stop')
                        continue
                else:
                    rospy.loginfo('All grasp pose tried, but fail to pick ' + str(self._target_pick_object))
        else:
            rospy.loginfo('target block pose illegal, No cartesian pose found corresponding to this block pose')
            rospy.loginfo('target block pose: ' + str_pose)

    def apply_action(self, state, action):
        print("enter apply action function")
        conf, holding, block_poses = state
        name, args = action
        if name == 'move':
            rospy.loginfo('moving')
            _, conf = args
            x, y = conf
            pose = np.array([x, y])
            print('target block pose: {}'.format(str(pose)))
            print('available block pose inverse: {}'.format(self._available_block_pose_inverse))
            if str(pose) in self._available_block_pose_inverse.keys():
                self._target_pick_object = self._available_block_pose_inverse[str(pose)]
                rospy.loginfo('Going to pick the object called: ' + str(self._target_pick_object))
            else:
                rospy.loginfo('Going to place the object called: ' + str(self._target_pick_object))
            self.mapping_move(str(pose))
        elif name == 'pick':
            rospy.loginfo('pick')
            holding, _, _ = args
            del block_poses[holding]
            self._motion_planner.pick()
            self._last_gripper_action = name
            print('{} is in hand now'.format(self._target_pick_object))
        elif name == 'place':
            rospy.loginfo('place')
            block, pose, _ = args
            holding = None
            block_poses[block] = pose
            print('nothing in hand')
            self._motion_planner.place()
            self._last_gripper_action = name
        elif name == 'push':
            block, _, _, pose, conf = args
            holding = None
        else:
            raise ValueError(name)
        return DiscreteTAMPState(conf, holding, block_poses)

    def apply_plan(self, tamp_problem, plan):
        print("enter apply plan function")
        state = tamp_problem.initial
        self._last_gripper_action = 'place'
        for action in plan:
            rospy.loginfo('action: ' + action.name)
            state = self.apply_action(state, action)
        self._motion_planner.to_home_pose()

    def task_planner_callback(self, data):
        print('enter listener callback function')
        self.once_plan_all()
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


if __name__ == '__main__':
    rospy.init_node('task_planner', anonymous=True)
    task_planner = TaskPlanner()
    alpha = 110 * math.pi / 180
    test_pose = Pose()
    test_pose.position.x = 0.1
    test_pose.position.y = 0.2
    test_pose.position.z = 0.3
    test_pose.orientation.x = 0
    test_pose.orientation.y = 0.25881904510252074
    test_pose.orientation.z = 0
    test_pose.orientation.w = 0.9659258262890683
    result = task_planner.get_artificial_intelligence_grasp_pose(test_pose)
    print('result: {}'.format(result))
    # while not rospy.is_shutdown():
    #     rospy.loginfo('Planner ready, waiting for trigger message!')
    #     rospy.spin()
