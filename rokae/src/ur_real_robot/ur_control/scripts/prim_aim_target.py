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
    def get_tgt_pose_in_world_frame(self,all_info):
        tool_len = 0.5
        tgt_pose_in_real_frame = geometry_msgs.msg.Pose()
        tgt_pose_in_real_frame.position.x = 0
        tgt_pose_in_real_frame.position.y = 0
        tgt_pose_in_real_frame.position.z = -tool_len-0.15

        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        tgt_pose_in_real_frame.orientation.x = q[0]
        tgt_pose_in_real_frame.orientation.y = q[1]
        tgt_pose_in_real_frame.orientation.z = q[2]
        tgt_pose_in_real_frame.orientation.w = q[3]
        tgt_pose_in_world_frame = self.transform_pose("real_bolt_frame",
                            "base_link",
                            tgt_pose_in_real_frame,
                            all_info['bolt_ts'])        
        # self.print_pose(tgt_pose_in_world_frame, 'tgt_pose_in_world_frame')
        print (tgt_pose_in_world_frame)
        (r, p, y) = tf.transformations.euler_from_quaternion([tgt_pose_in_world_frame.orientation.x, tgt_pose_in_world_frame.orientation.y, tgt_pose_in_world_frame.orientation.z, tgt_pose_in_world_frame.orientation.w])
        print(r,p,y)
        return tgt_pose_in_world_frame

    def action(self, all_info, pre_result_dict):
        for param in self.action_params:
            if not param in all_info.keys():
                print(param, 'must give')
                return False
        print("param satified, start to do mate")

        detect_ret=self.circle_detector.finish_YOLO_detect(all_info['rgb_img'])


        if 'screw' in detect_ret.keys():
            print('screw success')
            
            circles = detect_ret["screw"]
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
