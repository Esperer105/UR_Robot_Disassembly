#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import os
import threading

import tf
import sys
import cv2
import time
import rospy
import random
import pprint
import image_geometry
import message_filters
import numpy as np
from itertools import chain
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from tf import TransformListener, transformations
import testmotion
# from  bolt_position_detector
import templateMatching
import copy
import moveit_commander


# from PIL import Image,ImageDraw
# import numpy as np
from prim_aim_target import PrimAimTarget
from prim_clear_obstacle import PrimClearObstacle
from prim_insert import PrimInsert
from prim_move import PrimMove
import tf2_ros
import geometry_msgs.msg
#  新增import
from prim_action import PrimAction
from Queue import Queue


#修改部分   新规划方案
def auto_plan(original_stage):
    ori_stage=original_stage

    #建立动作原语集
    move=PrimAction('move')
    mate=PrimAction('mate')
    push=PrimAction('push')
    insert=PrimAction('insert')
    disassemble=PrimAction('disassemble')
    prim_list=[move,mate,push,insert,disassemble]

    #基于FIFO的规划生成方法
    pathQueue=Queue(0)
    pathQueue.put([ori_stage,[]])
    plan_is_end=False
    while not plan_is_end:
        tmp_pair=pathQueue.get()
        tmp_stage=tmp_pair[0]
        tmp_path=tmp_pair[1]
        if tmp_stage['disassembled']==True:
            pathQueue.put(tmp_pair)
            plan_is_end=True
        else:
            for primi in prim_list:
                if primi.able(tmp_stage)==True:
                    new_stage=primi.action(tmp_stage)
                    new_path=tmp_path[:]
                    new_path.append(primi.prim)
                    pathQueue.put([new_stage,new_path])
    path_list=[]
    while not pathQueue.empty():
        path=pathQueue.get()
        path_list.append(path[1])        
    
    
    #筛选出所有最短步数的规划方案
    min_step=100
    for path in path_list:
        if len(path)<min_step:
            min_step=len(path)
    path_list=[i for i in path_list if len(i)==min_step]
    return path_list


if __name__ == '__main__':
    stage={'have_coarse_pose':True, 'above_bolt':False,'target_aim':False, 'target_clear':False,'cramped':False,'disassembled':False}
    test_path_list=auto_plan(stage)
    for path in test_path_list:
        print  ('path: ')
        for step in path:
            print ('step: ', step)