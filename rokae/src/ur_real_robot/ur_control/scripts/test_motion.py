#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
import sys, random, copy
import rospy, tf, rospkg
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import *
from std_msgs.msg import *

import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
import math

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import  WrenchStamped

# rosmsg show  WrenchStamped.msg
#  rostopic echo  wrench




def robot_move_line(x1, y1, z1, x2, y2, z2):

    step = 5.0
    x_step_unit = (x2-x1)/step  # target_pose - start pose / move_step
    y_step_unit = (y2-y1)/step  # target_pose - start pose / move_step
    z_step_unit = (z2-z1)/step  # target_pose - start pose / move_step

    waypoints = []
    robot_pose = geometry_msgs.msg.Pose()
    now_pose = group1.get_current_pose().pose

    # robot move following vector
    for i in range(1, int(step)):
        now_pose.position.x = x1 + x_step_unit*i
        now_pose.position.y = y1 + y_step_unit*i
        now_pose.position.z = z1 + z_step_unit*i
        waypoints.append(copy.deepcopy(now_pose))

    # last target pose add in memory
    robot_pose.position.x = x2
    robot_pose.position.y = y2
    robot_pose.position.z = z2
    robot_pose.orientation.x = now_pose.orientation.x
    robot_pose.orientation.y = now_pose.orientation.y
    robot_pose.orientation.z = now_pose.orientation.z
    robot_pose.orientation.w = now_pose.orientation.w
    waypoints.append(copy.deepcopy(robot_pose))

    plan, fraction = group1.compute_cartesian_path(waypoints, 0.01, 0.0)
    group1.execute(plan, wait=True)


def robot_move_circle(x_temp, y_temp, z_temp):
    init_angle = 0
    delta_angle = 30
    scale_angle = delta_angle*math.pi/180
    radius = 0.05
    robot_pose = geometry_msgs.msg.Pose()
    now_pose = group1.get_current_pose().pose
    # robot move following vector
    # for i in range(360/delta_angle + 1):
    #     init_angle = +delta_angle * i

    #     tamp_angle = scale_angle * init_angle/delta_angle

    #     x_transform = radius*math.cos(tamp_angle)
    #     y_transform = radius*math.sin(tamp_angle)

    #     # print('x_transform{0},{1},{2}'.format(x_transform,init_angle,i))
    #     # print('y_transform{0},{1},{2}'.format(y_transform,init_angle,i))

    #     x_transform_distance = x_transform+x_temp
    #     y_transform_distance = y_transform+y_temp

    #     robot_move_line(now_pose.position.x, now_pose.position.y, now_pose.position.z,
    #                     x_transform_distance, y_transform_distance, now_pose.position.z)
    #     rospy.sleep(2)

    robot_move_line(now_pose.position.x, now_pose.position.y,
                    now_pose.position.z, x_temp, y_temp, z_temp)


if __name__ == "__main__":


    rospy.init_node("sort_robot_program")
    
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()

    group_name1 = "manipulator"
    group1 = moveit_commander.MoveGroupCommander(group_name1)
    group1.set_planner_id("RRTConnectkConfigDefault")

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(2)
    group1.set_named_target("up")    # Not use home for pose. it will load collision and  event
    plan1 = group1.plan()
    group1.execute(plan1,wait=True)

    rospy.sleep(2)
    
    print(Wrench.force)

    print(Wrench.torque)



    # for   i  in   Wrench.force:
        # print( i )
    # print(Wrench.torque)

    # test it robot move, must hold cb3 on hand,else not run script
    robot_move_circle(0.2, 0.3, 1.2)

    rospy.sleep(2)