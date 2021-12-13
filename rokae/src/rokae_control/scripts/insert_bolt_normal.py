#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
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
import concept_demo
import math
from time import sleep
from gazebo_msgs.srv import DeleteModel
import testmotion

import select
import termios
import tty

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import message_filters
import cv2
import image_geometry
import os
# import nsplanner
# import matplotlib.mlab as mlab
import numpy as np
# import matplotlib.pyplot as plt
import math
# import mpl_toolkits.mplot3d
from numpy import random

# from PIL import Image,ImageDraw
# import numpy as np
from prim_aim_target import PrimAimTarget
from prim_clear_obstacle import PrimClearObstacle
from prim_insert import PrimInsert
from prim_move import PrimMove
import tf2_ros
import geometry_msgs.msg
from visualization_msgs.msg import Marker
import threading
from nsplanner import NSPlanner
import matplotlib.pyplot as plt


def get_gazebo_model_pose():
    parts_pose = []
    model_pose = rospy.wait_for_message("gazebo/model_states", ModelStates)
    # for count in range(len(model_pose.name)-1):
    if len(model_pose.name) > 2:
        current_product = len(model_pose.name)-1
        name = model_pose.name[current_product]
        x = model_pose.pose[current_product].position.x
        y = model_pose.pose[current_product].position.y
        z = model_pose.pose[current_product].position.z

        return x,  y, z
    else:
        return 0, 0


def writelogs(write_data):
    # write_data.sort(key=takeSecond)
    # 打开文件
    file_name = 'random_deviation.txt'
    fo = open(file_name, 'a+')
    print("文件名为: ", fo.name)
    # for every in write_data:
    fo.write(write_data + "\n")
    fo.close()


def move_robot_nsplanner(planner,  x_offset, y_offset):
    x_pos_battery, y_pos_battery, z_pos_battery = get_gazebo_model_pose()

    quat = tf.transformations.quaternion_from_euler(-3.14, 0, 0)
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = x_pos_battery + x_bolt + x_offset
    pose_target.position.y = y_pos_battery + y_bolt + y_offset
    pose_target.position.z = z_bolt
    pose_target.orientation.x = quat[0]
    pose_target.orientation.y = quat[1]
    pose_target.orientation.z = quat[2]
    pose_target.orientation.w = quat[3]
    print('we have started.')
    start_success = False
    execute_frequency = 0
    while not start_success:
        print('++++++++++++++++++++++++++++++++++++++++++++++++++++')
        start_success = planner.start(pose_target)
        rospy.sleep(3)
    while not planner.is_stoping():
        print('wait for finished')
        rospy.sleep(3)
        if execute_frequency > 100:
            break
        execute_frequency += 1

    # now_pose = group1.get_current_pose().pose
    bolt_pose = planner.get_bolt_pose()

    # if (bolt_pose.position.x == now_pose.position.x and bolt_pose.position.y == now_pose.position.y) and (now_pose.position.z <= 1.2):
    #     start_success = True
    # else:
    #     start_success = False

    print('====================================================')
    print(pose_target)
    print(bolt_pose)
    print('====================================================')
    return start_success


def figure_show(x_datasets, normal_count, nsplanner_count):

    plt.title("planner demo")
    plt.xlabel("sigma distance")
    plt.ylabel("success rate")
    parameter_normal = np.polyfit(x_datasets, normal_count, 3)
    p_normal = np.poly1d(parameter_normal)

    plt.plot(x_datasets, p_normal(x_datasets), color='g', label='Traditional')

    parameter_our = np.polyfit(x_datasets, nsplanner_count, 3)
    p_our = np.poly1d(parameter_our)
    plt.plot(x_datasets, p_our(x_datasets),
             linewidth=2.0, color='blue', linestyle='-', label='Our')

    plt.legend()
    plt.show()


def bar_show(x_datasets, normal_bar, nsplanner_bar):

    # 并列柱状图
    plt.rcParams['font.sans-serif'] = ['SimHei']  # 设置字体以便支持中文
    x = np.arange(5)  # 柱状图在横坐标上的位置
    # 列出你要显示的数据，数据的列表长度与x长度相同
    y1 = [1, 3, 5, 4, 2]
    y2 = [2, 5, 3, 1, 6]

    bar_width = 0.001  # 设置柱状图的宽度
    tick_label = x_datasets

    # 绘制并列柱状图
    plt.bar(x_datasets, normal_bar, bar_width, color='b', label='Traditional')
    x_width = []
    xticks_label = []
    for i in range(len(x_datasets)):
        x_width.append(x_datasets[i]+bar_width)
        xticks_label.append(x_datasets[i]+bar_width/2)
    plt.bar(x_width, nsplanner_bar, bar_width, color='g', label='Our')

    plt.legend()  # 显示图例，即label
    # 显示x坐标轴的标签,即tick_label,调整位置，使其落在两个直方图中间位置
    plt.xticks(xticks_label, tick_label)
    plt.show()


def obstacle_deduce():

    file_name = 'random_deviation_obstacle.txt'
    fo = open(file_name, 'a+')
    size = 10
    deviation = 0.03
    mu = 0
    # datasets = []
    writelogs('x,y,semidiameter,normal_success,nsplanner_success')
    # datasets = [0.01, 0.02, 0.03, 0.04, 0.05]
    x_datasets = []
    normal_count = []
    nsplanner_count = []
    normal_bar = []
    nsplanner_bar = []
    for step in range(0, 10):
        is_probability = False
        is_success_nsplanner = False
        current_sigma = round(float(step)/100, 2)
        x_datasets.append(current_sigma)
        semidiameter = np.random.normal(loc=mu, scale=current_sigma, size=size)
        angle = np.random.randint(360, size=size)
        normal_num = 0
        nsplanner_num = 0
        for number in range(len(semidiameter)):
            x_current = abs(semidiameter[number]) * \
                math.cos(2 * math.pi * angle[number] / 360)
            y_current = abs(semidiameter[number]) * \
                math.sin(2 * math.pi * angle[number] / 360)

            if abs(semidiameter[number]) >= deviation:
                is_probability = True
                normal_num += 1

            print('current epoch is {} round {} sequence '.format(step, number+1))
            is_success_nsplanner = True
            print(angle[number])
            if float(angle[number]) >= 105 or float(angle[number]) <= 75:
                # if is_success_nsplanner:
                nsplanner_num += 1

            string = ('{},{},{},{},{}'.format(x_current, y_current,
                                              semidiameter[number], is_probability, is_success_nsplanner))
            writelogs(string)

            is_probability = False
            is_success_nsplanner = False

        normal_count.append(float(normal_num) / size)
        nsplanner_count.append(float(nsplanner_num)/size)

        normal_bar.append(float(normal_num * 3)/size)
        nsplanner_bar.append(float(float(nsplanner_num * 4)/size))

    figure_show(x_datasets, normal_count, nsplanner_count)

    bar_show(x_datasets, normal_bar, nsplanner_bar)

    writelogs('data_summary')
    for i in range(0, len(x_datasets)):
        string = ('{},{},{}'.format(
            x_datasets[i], normal_count[i], nsplanner_count[i]))
        writelogs(string)


def non_obstacle():

    deviation = 0.03
    mu = 0
    # datasets = []
    writelogs('x,y,semidiameter,normal_success,nsplanner_success')
    # datasets = [0.01, 0.02, 0.03, 0.04, 0.05]

    x_datasets = []
    normal_count = []
    nsplanner_count = []

    for step in range(0, 7):
        is_probability = False
        is_success_nsplanner = False
        current_sigma = round(float(step)/100, 2)
        x_datasets.append(current_sigma)
        semidiameter = np.random.normal(loc=mu, scale=current_sigma, size=10)
        angle = np.random.randint(360, size=10)
        normal_num = 0
        nsplanner_num = 0
        for number in range(len(semidiameter)):
            x_current = abs(semidiameter[number]) * \
                math.cos(2 * math.pi * angle[number] / 360)
            y_current = abs(semidiameter[number]) * \
                math.sin(2 * math.pi * angle[number] / 360)

            if abs(semidiameter[number]) <= deviation:
                is_probability = True
                normal_num += 1

            print('current epoch is {} round {} sequence '.format(step, number+1))
            is_success_nsplanner = move_robot_nsplanner(
                planner, x_current, y_current)
            if is_success_nsplanner:
                nsplanner_num += 1
            string = ('{},{},{},{},{}'.format(x_current, y_current,
                                              semidiameter[number], is_probability, is_success_nsplanner))
            writelogs(string)

            is_probability = False
            is_success_nsplanner = False

        normal_count.append(float(normal_num) / 10)
        nsplanner_count.append(float(nsplanner_num)/10)

    plt.title("planner demo")
    plt.xlabel("sigma distance")
    plt.ylabel("success rate")
    parameter = np.polyfit(x_datasets, normal_count, 3)
    p = np.poly1d(parameter)

    plt.plot(x_datasets, p(x_datasets), color='g', label='PDDL')

    plt.plot(x_datasets, normal_count, linewidth=2.0,
             color='red', linestyle='--', label='PDDL')
    plt.plot(x_datasets, nsplanner_count,
             linewidth=2.0, color='blue', linestyle='-', label='Our')
    plt.legend()

    plt.show()

    writelogs('data_summary')

    for i in range(0, len(x_datasets)):
        string = ('{},{},{}'.format(
            x_datasets[i], normal_count[i], nsplanner_count[i]))
        writelogs(string)


if __name__ == "__main__":

    rospy.init_node('nsplanner-moveit', anonymous=True)

    planner = NSPlanner('camera', '/camera/color/image_raw',
                        '/camera/depth/image_raw', '/camera/color/camera_info')
    # group_name1 = "arm"
    # group1 = moveit_commander.MoveGroupCommander(group_name1)
    x_bolt = -0.057323   # 数值大，向下
    y_bolt = 0.03838  # 数值大，向左
    z_bolt = 1.3

    print('please choose model for non_obstacle or has obstacle. obstacle模式输入o，non_obstacle模式输入n')
    input_model = raw_input()

    if input_model == 'o':
        print('测试有障碍物的模式')
        obstacle_deduce()
    else:
        print('测试无障碍物的模式')
        non_obstacle()

    # deviation = 0.03
    # mu = 0
    # # datasets = []
    # writelogs('x,y,semidiameter,normal_success,nsplanner_success')
    # # datasets = [0.01, 0.02, 0.03, 0.04, 0.05]

    # x_datasets = []
    # normal_count = []
    # nsplanner_count = []

    # for step in range(1, 7):
    #     is_probability = False
    #     is_success_nsplanner = False
    #     current_sigma = round(float(step)/100, 2)
    #     x_datasets.append(current_sigma)
    #     semidiameter = np.random.normal(loc=mu, scale=current_sigma, size=10)
    #     angle = np.random.randint(360, size=10)
    #     normal_num = 0
    #     nsplanner_num = 0
    #     for number in range(len(semidiameter)):
    #         x_current = abs(semidiameter[number]) * \
    #             math.cos(2 * math.pi * angle[number] / 360)
    #         y_current = abs(semidiameter[number]) * \
    #             math.sin(2 * math.pi * angle[number] / 360)

    #         if abs(semidiameter[number]) <= deviation:
    #             is_probability = True
    #             normal_num += 1

    #         print('current epoch is {} round {} sequence '.format(step, number+1))
    #         is_success_nsplanner = move_robot_nsplanner(
    #             planner, x_current, y_current)
    #         if is_success_nsplanner:
    #             nsplanner_num += 1
    #         string = ('{},{},{},{},{}'.format(x_current, y_current,
    #                                           semidiameter[number], is_probability, is_success_nsplanner))
    #         writelogs(string)

    #         is_probability = False
    #         is_success_nsplanner = False

    #     normal_count.append(float(normal_num) / 10)
    #     nsplanner_count.append(float(nsplanner_num)/10)

    # plt.title("planner demo")
    # plt.xlabel("sigma distance")
    # plt.ylabel("success rate")
    # parameter = np.polyfit(x_datasets, normal_count, 3)
    # p = np.poly1d(parameter)

    # plt.plot(x_datasets, p(x_datasets), color='g', label='PDDL')

    # plt.plot(x_datasets, normal_count, linewidth=2.0,
    #          color='red', linestyle='--', label='PDDL')
    # plt.plot(x_datasets, nsplanner_count,
    #          linewidth=2.0, color='blue', linestyle='-', label='Our')
    # plt.legend()

    # plt.show()

    # writelogs('data_summary')

    # for i in range(0, len(x_datasets)):
    #     string = ('{},{},{}'.format(
    #         x_datasets[i], normal_count[i], nsplanner_count[i]))
    #     writelogs(string)

    while not rospy.is_shutdown():
        rospy.spin()

    # fo.close()
