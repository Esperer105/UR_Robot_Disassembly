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
import math

from PIL import Image
# from PIL import Image,ImageDraw
# import numpy as np 
from bolt_detector import BoltDetector
from rigid_transform_3D import rigid_transform_3D
from test_base import TestBase

class TestAimTarget(TestBase):
    def get_tgt_pose_in_world_frame(self,all_info):
        tool_len = 0.42
        tgt_pose_in_real_frame = geometry_msgs.msg.Pose()
        tgt_pose_in_real_frame.position.x = 0
        tgt_pose_in_real_frame.position.y = 0
        tgt_pose_in_real_frame.position.z = - tool_len-0.05

        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        tgt_pose_in_real_frame.orientation.x = q[0]
        tgt_pose_in_real_frame.orientation.y = q[1]
        tgt_pose_in_real_frame.orientation.z = q[2]
        tgt_pose_in_real_frame.orientation.w = q[3]
        tgt_pose_in_world_frame = self.transform_pose("real_bolt_frame",
                                                      "base_link",
                                                      tgt_pose_in_real_frame,
                                                      all_info['bolt_ts'] )
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
        print("param satified, start to mate")
        planner = all_info['planner_handler']
        np_collected=False
        while not kalman.finished:
            latest_infos = planner.get_latest_infos()
            # print (latest_infos.keys())        
            raw_img=latest_infos['rgb_img']
            height=raw_img.shape[0]
            width =raw_img.shape[1]
            r_height=540
            r_width =960
            # print(height,width)
            crop_img= cv2.copyMakeBorder(raw_img,(r_height-height)/2,(r_height-height)/2,(r_width-width)/2,(r_width-width)/2,cv2.BORDER_CONSTANT,value=0)
            # crop_img=raw_img[int(0.25*height):int(0.75*height),int(0.5*(width-0.5*height)):int(0.5*(width+0.5*height))]
            # crop_img=raw_img[:,int(0.5*(width-height)):int(0.5*(width+height))]
            detect_ret=yolo.finish_YOLO_detect(crop_img)
            s=kalman.itr_sum
            if 'screw' in detect_ret[1].keys() or 'nut' in detect_ret[1].keys():
                circlesbox=[]
                if 'screw' in detect_ret[1].keys():
                    print('screw success')
                    circlesbox.extend(detect_ret[1]["screw"])
                if 'nut' in detect_ret[1].keys():
                    print('nut success')
                    circlesbox.extend(detect_ret[1]["nut"])                
                #circle = self.findBestMatchCircle(circles)

                # x = circle[1]+int(0.5*(width-0.5*height))
                # y = circle[0]+int(0.25*height)
                # x=circle[1]+int(0.5*(width-height))
                # y=circle[0]
                if (s==0):
                    # circle = self.findBestMatchCircle(circles) 
                    min_dist=100
                    curr_pose= self.group.get_current_pose(self.effector).pose
                    for screw in circlesbox:
                        screw[0] = screw[0]-(r_width-width)/2
                        screw[2] = screw[2]-(r_width-width)/2
                        screw[1] = screw[1]-(r_height-height)/2
                        screw[3] = screw[3]-(r_height-height)/2                        
                        self.add_bolt_frameV2(screw, latest_infos)
                        screw_pose=self.get_bolt_pose_in_world_frame(latest_infos)
                        temp_dist=math.sqrt(pow(screw_pose.position.x - curr_pose.position.x ,2)+pow(screw_pose.position.y - curr_pose.position.y ,2))            
                        if (temp_dist<min_dist):
                            min_dist=temp_dist
                            conv_pose=screw_pose
                    real_pose=kalman.iteration(conv_pose)
                    self.adjust_bolt_frame(real_pose,latest_infos)
                    ee_pose=self.get_tgt_pose_in_world_frame(latest_infos)

                    if not self.set_arm_pose(self.group, ee_pose, self.effector):
                        print("failed")
                        print(curr_pose)
                else:
                    min_diff=100
                    for screw in circlesbox:
                        # self.add_bolt_frame(screw[1]-(r_width-width)/2, screw[0]-(r_height-height)/2, latest_infos)
                        screw[0] = screw[0]-(r_width-width)/2
                        screw[2] = screw[2]-(r_width-width)/2
                        screw[1] = screw[1]-(r_height-height)/2
                        screw[3] = screw[3]-(r_height-height)/2
                        self.add_bolt_frameV2(screw, latest_infos)
                        screw_pose=self.get_bolt_pose_in_world_frame(latest_infos)
                        former_pose=kalman.get_former_pose()
                        # temp_diff=math.sqrt(pow(screw_pose.position.x - former_pose.position.x ,2)+pow(screw_pose.position.y - former_pose.position.y ,2)+pow(screw_pose.position.z- former_pose.position.z,2))
                        temp_diff=math.sqrt(pow(screw_pose.position.x - former_pose.position.x ,2)+pow(screw_pose.position.y - former_pose.position.y ,2))                              
                        if (temp_diff<min_diff):
                            min_diff=temp_diff
                            near_pose=screw_pose
                        if(temp_diff > 0.05) and (np_collected==False):
                            coarse_pose = geometry_msgs.msg.Pose()
                            if  screw_pose.position.x >0 and  screw_pose.position.x <0.02 :
                                coarse_pose.position.x=0.08
                            else:    
                                coarse_pose.position.x = screw_pose.position.x-0.02
                            coarse_pose.position.y = screw_pose.position.y-0.02
                            coarse_pose.position.z = 0.70

                            q = tf.transformations.quaternion_from_euler(-math.pi, 0, 0.5*math.pi)
                            coarse_pose.orientation.x = q[0]
                            coarse_pose.orientation.y = q[1]
                            coarse_pose.orientation.z = q[2]
                            coarse_pose.orientation.w = q[3]

                            planner.next_pose=coarse_pose
                            np_collected=True

                    if (min_diff < 0.015):
                        real_pose=kalman.iteration(near_pose)
                        self.adjust_bolt_frame(real_pose,latest_infos)
                        ee_pose=self.get_tgt_pose_in_world_frame(latest_infos)
                        curr_pose= self.group.get_current_pose(self.effector).pose
                        if not self.set_arm_pose(self.group, ee_pose, self.effector):
                            print("failed")
                            print(curr_pose)
                    else:
                        if not self.set_arm_pose(self.group, curr_pose, self.effector):
                            print("recovery failed")
            else:
                if (s==0):
                    curr_pose= self.group.get_current_pose(self.effector).pose
                if not self.set_arm_pose(self.group, curr_pose, self.effector):
                    print("recovery failed")
                    # curr_pose= self.group.get_current_pose(self.effector).pose
                    # print(curr_pose)                
        if not real_pose is None:
            print('real pose')
            print(real_pose)
            (r, p, y) = tf.transformations.euler_from_quaternion([real_pose.orientation.x, real_pose.orientation.y, real_pose.orientation.z, real_pose.orientation.w])
            print(r,p,y)
            
            return {'success': True, 'bolt_pose': real_pose}            
        else:
            print ('location failed')
            return {'success': False}