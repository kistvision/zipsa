#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import math
import actionlib
from living_lab_robot_moveit_client.msg import PlanExecutePoseConstraintsAction, PlanExecutePoseConstraintsGoal
from tf.transformations import *
from moveit_msgs.msg import JointConstraint
from std_srvs.srv import Empty
import time

def main(argv):
    rospy.wait_for_service('/clear_octomap') #this will stop your code until the clear octomap service starts running
    clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)

    client = actionlib.SimpleActionClient('/plan_and_execute_pose_w_joint_constraints', PlanExecutePoseConstraintsAction)
    client.wait_for_server()

    goal = PlanExecutePoseConstraintsGoal()
    try:
        goal.target_pose.header.frame_id = argv[0]
        goal.target_pose.pose.position.x = float(argv[1])
        goal.target_pose.pose.position.y = float(argv[2])
        goal.target_pose.pose.position.z = float(argv[3])

        goal.target_pose.pose.orientation.x = float(argv[4])
        goal.target_pose.pose.orientation.y = float(argv[5])
        goal.target_pose.pose.orientation.z = float(argv[6])
        goal.target_pose.pose.orientation.w = float(argv[7])

    except ValueError:
        quit()


    # start_time = time.time()
    clear_octomap()
    # end_time = time.time()
    # print(end_time - start_time)
    client.send_goal(goal)
    client.wait_for_result()

    print(client.get_result())

if __name__ == '__main__':
    rospy.init_node('test_pose', anonymous=False)

    # if len(sys.argv) != 8:
    #     print "Usage: rosrun living_lab_robot_moveit_client test_pose <reference_link> <x> <y> <z> <r (deg)> <p (deg)> <y (deg)>"
    #     exit(-1)

    m = main(sys.argv[1:])

    # rospy.spin()