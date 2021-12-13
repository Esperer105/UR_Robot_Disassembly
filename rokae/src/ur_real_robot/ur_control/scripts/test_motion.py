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
import  force_wrench






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


def robot_move_location():
    
    group_name1 = "manipulator"
    group1 = moveit_commander.MoveGroupCommander(group_name1)
    group1.set_planner_id("RRTConnectkConfigDefault")

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
    scene = moveit_commander.PlanningSceneInterface()
    
    group1.set_goal_joint_tolerance(0.001)
     # 设置允许的最大速度和加速度，0.5是一个比例，乘以允许的最大速度和加速度
    group1.set_max_acceleration_scaling_factor(0.5)
    group1.set_max_velocity_scaling_factor(0.5)

    rospy.sleep(2)
    robot_move(group1, -0.092233, 0.50576, 0.84384, -0.23767, 0.97122, 0,0)


def robot_move(group, x, y, z, R, P, Y,W):
    # moveit_commander.roscpp_initialize(sys.argv)
    # robot = moveit_commander.RobotCommander()

    group.clear_pose_targets()
    # quat = tf.transformations.quaternion_from_euler(R, P, Y)
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z
    pose_target.orientation.x = R
    pose_target.orientation.y = P
    pose_target.orientation.z =Y
    pose_target.orientation.w = W
    group.set_pose_target(pose_target)
    plan = group.plan()
    display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)

    group.execute(plan, wait=True)
    print(("test %f,%f,%f,%f,%f,%f") % (x, y, z, R, P, Y,W))


def load_collision_box():

    # uodo   动态加载 碰撞盒位置
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(2)
    box_pose = geometry_msgs.msg.PoseStamped()
    scene.remove_world_object("box1")
    box_pose.header.frame_id = robot.get_planning_frame()
    box_pose.pose.position.x = 0.05
    box_pose.pose.position.y = 0.05
    box_pose.pose.position.z = 0
    box_pose.pose.orientation.w = 1.0
    # box_pose.pose.position.z = 1 # slightly above the end effector
    scene.add_box("box1", box_pose, size=(3, 3, 0.01))
    
    

def MoveItFkDemo():

    group_name1 = "manipulator"
    group1 = moveit_commander.MoveGroupCommander(group_name1)
    group1.set_planner_id("RRTConnectkConfigDefault")

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
    scene = moveit_commander.PlanningSceneInterface()
    
    group1.set_goal_joint_tolerance(0.001)
     # 设置允许的最大速度和加速度，0.5是一个比例，乘以允许的最大速度和加速度
    group1.set_max_acceleration_scaling_factor(0.05)
    group1.set_max_velocity_scaling_factor(0.05)

    rospy.sleep(2)
    group1.set_named_target("up")    # Not use home for pose. it will load collision and  event
    plan1 = group1.plan()

    group1.execute(plan1,wait=True)
    group1.stop()

    rospy.sleep(2)
    
    
    
def roate_value(group1)   :
    joint_goal = group1.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -math.pi/2
    joint_goal[2] = 0
    joint_goal[3] = -math.pi/2
    joint_goal[4] = 0
    joint_goal[5] = math.pi*2
    return joint_goal

    
    
    
def MoveIt_position():

    group_name1 = "manipulator"
    group1 = moveit_commander.MoveGroupCommander(group_name1)
    group1.set_planner_id("RRTConnectkConfigDefault")

    group1.set_goal_joint_tolerance(0.001)
     # 设置允许的最大速度和加速度，0.5是一个比例，乘以允许的最大速度和加速度
    group1.set_max_acceleration_scaling_factor(0.05)
    group1.set_max_velocity_scaling_factor(0.05)

    group1.clear_pose_targets()
    # quat = tf.transformations.quaternion_from_euler(R, P, Y)
    pose_target = geometry_msgs.msg.Pose()
    # joint_goal = group1.get_current_joint_values()

    # parameters if you have already set the pose or joint target for the group

    roate_num=2
    joint_goal=roate_value(group1)
    print('current roate_num {0}',roate_num)
    suceed=group1.go(joint_goal, wait=False)
    print('print')
    # force_wrench.pose_subscriber()
        # group1.stop()
        # group1.clear_pose_targets()
        # moveit_commander.roscpp_shutdown()
        # moveit_commander.os._exit(0)
        # group1.stop()

        # group1.clear_pose_target()
        
    # Calling ``stop()`` ensures that there is no residual movement


def load_collision_battery_pack_box():

    # uodo   动态加载 碰撞盒位置
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(2)
    box_pose = geometry_msgs.msg.PoseStamped()
    scene.remove_world_object("battery_pack")
    box_pose.header.frame_id = robot.get_planning_frame()
    box_pose.pose.position.x = 0.3
    box_pose.pose.position.y =0.82
    box_pose.pose.position.z = 0.27
    box_pose.pose.orientation.w = 1.0
    # box_pose.pose.position.z = 1 # slightly above the end effector
    scene.add_box("battery_pack", box_pose, size=(3, 0.5,0.278))
    
    
    


if __name__ == "__main__":


    rospy.init_node("sort_robot_program")
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    load_collision_box()
    load_collision_battery_pack_box()

    MoveItFkDemo()   # load to UP position
    MoveIt_position()


    # test it robot move, must hold cb3 on hand,else not run script
    # robot_move_circle(-0.092233, 0.50576, 0.84384)

    rospy.sleep(2)