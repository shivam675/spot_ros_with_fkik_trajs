#! /usr/bin/env python3
from __future__ import division
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import actionlib
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np


    # Constructor
class LegMoveit:
    def __init__(self, controller_name):
        # rospy.init_node('controller_{}'.format(controller_name), anonymous=True)
        self._planning_group = controller_name
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m') 
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_pose(self, arg_pose):
        # self._planning_group
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan
    
    def go_to_defined_pose(self, Plan_group, arg_pose_name):
        '''prefined pose combined with plan_group to minimise error '''
        self._planning_group = Plan_group
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._group.set_named_target(arg_pose_name)
        rospy.sleep(1)
        # plan_success, plan, planning_time, error_code = self._group.plan() 
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        rospy.sleep(1)
        self._exectute_trajectory_client.wait_for_result()

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')



def main():
    
    # front_left = LegMoveit('front_left')
    front_right = LegMoveit('front_right')
    # rear_left = LegMoveit('rear_left')
    # rear_right = LegMoveit('rear_right')

    # waypoints = []

    first_quad = geometry_msgs.msg.Pose()
    first_quad.position.x = 0.01
    first_quad.position.y = 0.01
    first_quad.position.z = 0.01
    first_quad.orientation.x = 0.0013
    first_quad.orientation.y = 0.3225
    first_quad.orientation.z = 0.0004
    first_quad.orientation.w = 0.9465
    # waypoints.append(copy.deepcopy(first_quad)) # Init pose

    # second_quad = geometry_msgs.msg.Pose()
    # second_quad.position.x = 0.2
    # second_quad.position.y = -0.102
    # second_quad.position.z = -0.057
    # # second_quad.orientation.x = 0.0013
    # # second_quad.orientation.y = 0.3225
    # # second_quad.orientation.z = 0.0004
    # # second_quad.orientation.w = 0.9465
    # waypoints.append(copy.deepcopy(second_quad))


    # print(len(waypoints))
    # for _ in range(10):
    # front_left.go_to_pose(first_quad)
    front_right.go_to_pose(first_quad)
    # rear_left.go_to_pose(first_quad)
    # rear_right.go_to_pose(first_quad)
    # print(waypoints)
    # aRm.go_to_pose(waypoints)

    # del front_left, front_right, rear_left, rear_right

if __name__ == '__main__':
    # while not rospy.is_shutdown():
    rospy.init_node('python_Controllers', anonymous=True )
    main()