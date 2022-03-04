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

from rigid_transform_3D import rigid_transform_3D
from prim_base import PrimBase
from bolt_detector import BoltDetector

class PrimAimTarget(PrimBase):
    def get_tgt_pose_in_world_frame(self, all_info):
        tgt_pose_in_bolt_frame = geometry_msgs.msg.Pose()
        tgt_pose_in_bolt_frame.position.x = -0.5
        tgt_pose_in_bolt_frame.position.y = 0
        tgt_pose_in_bolt_frame.position.z = 0
        # q = tf.transformations.quaternion_from_euler(0, 1.57, 0)
        q = tf.transformations.quaternion_from_euler(0, 1.57, 0)
        tgt_pose_in_bolt_frame.orientation.x = q[0]
        tgt_pose_in_bolt_frame.orientation.y = q[1]
        tgt_pose_in_bolt_frame.orientation.z = q[2]
        tgt_pose_in_bolt_frame.orientation.w = q[3]
        # self.print_pose(tgt_pose_in_bolt_frame, 'tgt_pose_in_bolt_frame')
        tgt_pose_in_world_frame = self.transform_pose("bolt_frame",
                                                      "base",
                                                      tgt_pose_in_bolt_frame,
                                                      all_info['timestamp'])
        # self.print_pose(tgt_pose_in_world_frame, 'tgt_pose_in_world_frame')
        return tgt_pose_in_world_frame

    def action(self, all_info, pre_result_dict):
        for param in self.action_params:
            if not param in all_info.keys():
                print(param, 'must give')
                return False
        print("param satified, start to do mate")

        detect_ret=(self.circle_detector.detect(all_info['rgb_img'],threshold=0.8))


        if 'circles' in detect_ret.keys():
            print('circle success')
            
            circles = detect_ret["circles"]
            circle = self.findBestMatchCircle(circles)

            x = circle[0]
            y = circle[1]
            self.add_bolt_frame(x, y, all_info)

            bolt_pose = self.get_bolt_pose_in_world_frame(all_info)
            ee_pose = self.get_tgt_pose_in_world_frame(all_info)
            if  not ee_pose is None:
                if not self.set_arm_pose(self.group, ee_pose, self.effector):
                    ee_pose = self.group.get_current_pose(self.effector).pose
                self.print_pose(ee_pose)
                return {'success': True, 'bolt_pose': bolt_pose}  
            
        return {'success': False}