





import random
import copy
import rospy
import tf
import rospkg
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import *
from std_msgs.msg import *

import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
import math
from time import sleep
from gazebo_msgs.srv import DeleteModel

import select
import termios
import tty

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import message_filters
import cv2
import image_geometry
import os
import logging
import numpy as np
import matplotlib.pyplot as plt


def set_arm_pose(group, pose, effector):
    group.set_goal_joint_tolerance(0.001)
     # 设置允许的最大速度和加速度，0.5是一个比例，乘以允许的最大速度和加速度
    group.set_max_acceleration_scaling_factor(0.05)
    group.set_max_velocity_scaling_factor(0.05)
    
    group.set_pose_target(pose, effector)
    plan = group.plan()
    if len(plan.joint_trajectory.points) > 0:
        group.execute(plan, wait=True)
    else:
        print ('no plan result')



def set_align_vertical_capture(ee_pose):
    # 相机旋转
    
    delta_rpy_random = random.randint(-314, 314)
    delta_rpy_random = float(delta_rpy_random)/float(400.0)
    # delta_rpy_random=0.02

    q = (ee_pose.orientation.x, ee_pose.orientation.y,
         ee_pose.orientation.z, ee_pose.orientation.w)
    rpy = tf.transformations.euler_from_quaternion(q)

    # rpy:变换
    # q = tf.transformations.quaternion_from_euler(
    #     -math.pi, rpy[1]+delta_rpy_random, rpy[2])
    
    q = tf.transformations.quaternion_from_euler(
        -math.pi, rpy[1]+math.pi/8, rpy[2])
    ee_pose.orientation.x = q[0]
    ee_pose.orientation.y = q[1]
    ee_pose.orientation.z = q[2]
    ee_pose.orientation.w = q[3]

    # set_arm_pose(group, ee_pose, effector)
    set_arm_pose(group, ee_pose, effector)




def print_pose(pose):
    q = (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w)
    rpy = tf.transformations.euler_from_quaternion(q)
    print('%s: position (%.2f %.2f %.2f) orientation (%.2f %.2f %.2f %.2f) RPY (%.2f %.2f %.2f)' %
          (effector, pose.position.x, pose.position.y, pose.position.z,
           pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
           rpy[0], rpy[1], rpy[2]))




if __name__ == "__main__":



    effector = sys.argv[1] if len(sys.argv) > 1 else 'wrist_3_link'

    settings = termios.tcgetattr(sys.stdin)
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('teleop_end_effector', anonymous=True)
    # group = moveit_commander.MoveGroupCommander("xarm6")
    group = moveit_commander.MoveGroupCommander("manipulator")
    group.set_planner_id("RRTConnectkConfigDefault")


    ee_pose = group.get_current_pose(effector).pose
    print_pose(ee_pose)
    for  i  in  range  ( 0,10):
        set_align_vertical_capture(ee_pose)