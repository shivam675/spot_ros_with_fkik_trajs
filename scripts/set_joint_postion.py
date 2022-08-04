#! /usr/bin/env python3

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math


class LegCommander:

    # Constructor
    def __init__(self, controller_name):

        rospy.init_node('set_joint_angles_node', anonymous=True)

        self._planning_group = controller_name
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()


    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo(list_joint_values)
        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        return flag_plan

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class aRmMoveit Deleted." + '\033[0m')


def main():

    front_left_leg = LegCommander('front_left')

    lst_joint_angles_1 = [0, 0.615, 2.46]

    lst_joint_angles_5 = [math.radians(20),
                          math.radians(-20),
                          math.radians(-20),]



    # while not rospy.is_shutdown():
    front_left_leg.set_joint_angles(lst_joint_angles_1)
    rospy.sleep(0.005)
    front_left_leg.set_joint_angles(lst_joint_angles_5)
    rospy.sleep(0.005)
    del front_left_leg


if __name__ == '__main__':
    main()