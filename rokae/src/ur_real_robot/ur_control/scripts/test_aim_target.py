#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import importlib
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
# from  bolt_position_detector
import copy
import tf2_ros
import geometry_msgs.msg
import traceback
import random

# from PIL import Image,ImageDraw
# import numpy as np 
from bolt_detector import BoltDetector
from rigid_transform_3D import rigid_transform_3D
from test_base import TestBase

class TestAimTarget(TestBase):
    def get_tgt_pose_in_world_frame(self,all_info):
        tool_len = 0.5
        tgt_pose_in_real_frame = geometry_msgs.msg.Pose()
        tgt_pose_in_real_frame.position.x = 0
        tgt_pose_in_real_frame.position.y = 0
        tgt_pose_in_real_frame.position.z = - tool_len
        # q = tf.transformations.quaternion_from_euler(0, 1.57, 0)
        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        tgt_pose_in_real_frame.orientation.x = q[0]
        tgt_pose_in_real_frame.orientation.y = q[1]
        tgt_pose_in_real_frame.orientation.z = q[2]
        tgt_pose_in_real_frame.orientation.w = q[3]
        # self.print_pose(tgt_pose_in_bolt_frame, 'tgt_pose_in_bolt_frame')
        tgt_pose_in_world_frame = self.transform_pose("real_bolt_frame",
                                                      "base_link",
                                                      tgt_pose_in_real_frame,
                                                      all_info['bolt_ts']
                                                      )
        # self.print_pose(tgt_pose_in_world_frame, 'tgt_pose_in_world_frame')
        print (tgt_pose_in_world_frame)
        (r, p, y) = tf.transformations.euler_from_quaternion([tgt_pose_in_world_frame.orientation.x, tgt_pose_in_world_frame.orientation.y, tgt_pose_in_world_frame.orientation.z, tgt_pose_in_world_frame.orientation.w])
        print(r,p,y)
        return tgt_pose_in_world_frame

    def action(self, all_info, pre_result_dict,kalman):
        for param in self.action_params:
            if not param in all_info.keys():
                print(param, 'must give')
                return False
        print("param satified, start to do mate")

        detect_ret=self.circle_detector.detect_edge_box(all_info['rgb_img'])

        if 'circles' in detect_ret.keys():
            print('circle success')
            
            circles = detect_ret["circles"]
            circle = self.findBestMatchCircle(circles)

            x = circle[0]
            y = circle[1]
            self.add_bolt_frame(x, y, all_info)

            bolt_pose = self.get_bolt_pose_in_world_frame(all_info)
            real_pose=kalman.iteration(bolt_pose)
            self.adjust_bolt_frame(real_pose,all_info)
            real_bolt_pose = geometry_msgs.msg.Pose()
            real_bolt_pose.position.x = real_pose[0,0]
            real_bolt_pose.position.y = real_pose[1,0]
            real_bolt_pose.position.z = real_pose[2,0]
            real_bolt_pose.orientation.x = real_pose[3,0]
            real_bolt_pose.orientation.y = real_pose[4,0]
            real_bolt_pose.orientation.z = real_pose[5,0]
            real_bolt_pose.orientation.w= real_pose[6,0]
            print(real_bolt_pose)
            (r, p, y) = tf.transformations.euler_from_quaternion([real_bolt_pose.orientation.x, real_bolt_pose.orientation.y, real_bolt_pose.orientation.z, real_bolt_pose.orientation.w])
            print(r,p,y)
            curr_pose=self.group.get_current_pose(self.effector).pose
            ee_pose = self.get_tgt_pose_in_world_frame(all_info)
            print('aim_pose')
            print(ee_pose)
            (r, p, y) = tf.transformations.euler_from_quaternion([real_bolt_pose.orientation.x, real_bolt_pose.orientation.y, real_bolt_pose.orientation.z, real_bolt_pose.orientation.w])
            print(r,p,y)

            if not self.set_arm_pose(self.group, ee_pose, self.effector):
                print('failed')
            
            ee_pose = curr_pose
            
            if not self.set_arm_pose(self.group, ee_pose, self.effector):
                ee_pose = self.group.get_current_pose(self.effector).pose
            self.print_pose(ee_pose)
            return {'success': True, 'bolt_pose': real_bolt_pose}  
        
        return {'success': False}