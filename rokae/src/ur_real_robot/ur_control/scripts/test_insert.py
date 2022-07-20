#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from test_base import TestBase
import math
import geometry_msgs.msg
import tf
import rospy
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
from kalman import  Kalman
import math

class TestInsert(TestBase):
    def get_insert_trajectory(self,real_pose,all_info):

        trajectory =  [] 
        radius=0.0015
        delta_angle = 30
        scale_angle = delta_angle * math.pi / 180
        scale_depth= 0.002

        tool_len = 0.435
        print('get_insert_trajectory')
        for i in range(180 / delta_angle + 1):
            tamp_radius=radius*(1-i*delta_angle/180)
            tamp_angle = -scale_angle * i
            tamp_depth=scale_depth * i
            # SJTU HERE CHANGED ori: z x y
            tgt_pose_in_real_frame = geometry_msgs.msg.Pose()
            tgt_pose_in_real_frame.position.x = tamp_radius * math.cos(tamp_angle)
            tgt_pose_in_real_frame.position.y = 0.004+ tamp_radius * math.sin(tamp_angle)
            tgt_pose_in_real_frame.position.z = -tool_len+tamp_depth
            q = tf.transformations.quaternion_from_euler(0, 0, tamp_angle)
            tgt_pose_in_real_frame.orientation.x = q[0]
            tgt_pose_in_real_frame.orientation.y = q[1]
            tgt_pose_in_real_frame.orientation.z = q[2]
            tgt_pose_in_real_frame.orientation.w = q[3]
            
            tgt_pose_in_world_frame = self.transform_pose("real_bolt_frame",
                                                          "base_link",
                                                          tgt_pose_in_real_frame,
                                                          all_info['bolt_ts'])
            print (tgt_pose_in_world_frame)
            (r, p, y) = tf.transformations.euler_from_quaternion([tgt_pose_in_world_frame.orientation.x, tgt_pose_in_world_frame.orientation.y, tgt_pose_in_world_frame.orientation.z, tgt_pose_in_world_frame.orientation.w])
            print(r,p,y)

            if not tgt_pose_in_world_frame is None:
                trajectory.append(tgt_pose_in_world_frame)
                print ("the %d-th trajectory"%(i))
        if len(trajectory) > 0:
            print ("trajectory collected")
        return trajectory
    
    def get_tgt_pose_in_world_frame(self,all_info):
        tool_len = 0.435
        tgt_pose_in_real_frame = geometry_msgs.msg.Pose()
        tgt_pose_in_real_frame.position.x = 0
        tgt_pose_in_real_frame.position.y = 0.004
        tgt_pose_in_real_frame.position.z = -tool_len

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

    def action(self, all_info, pre_result_dict,kalman,yolo):
        for param in self.action_params:
            if not param in all_info.keys():
                print(param, 'must give')
                return False
        print("param satified, start to insert")
        
        if not kalman.get_former_pose()  is None:
            real_pose=kalman.get_former_pose()    
            print('real bolt detected')
            print('real pose')
            print(real_pose)
            (r, p, y) = tf.transformations.euler_from_quaternion([real_pose.orientation.x, real_pose.orientation.y, real_pose.orientation.z, real_pose.orientation.w])
            print(r,p,y)
            self.adjust_bolt_frame(real_pose,all_info)
            ee_pose=self.get_tgt_pose_in_world_frame(all_info)
            curr_pose= self.group.get_current_pose(self.effector).pose
            if not self.set_arm_pose(self.group, ee_pose, self.effector):
                print("failed")
                print(curr_pose)   
            trajectory=self.get_insert_trajectory(real_pose,all_info)
            for ee_pose in trajectory:
                if not self.set_arm_pose(self.group, ee_pose, self.effector):
                    print("insert failed")
                    ee_pose = self.group.get_current_pose(self.effector).pose 
                    print(ee_pose)
            return {'success': True}
        return {'success': False}