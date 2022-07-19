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


class TestInsert(TestBase):
    def add_real_frame(self, kalman, all_info):
        real_trans = geometry_msgs.msg.TransformStamped()
        real_trans.header.stamp = rospy.Time.now()
        all_info['bolt_ts']=real_trans.header.stamp
        print("real_broadcast_tf")
        print(real_trans.header.stamp)
        real_trans.header.frame_id = "base_link"
        real_trans.child_frame_id = "real_bolt_frame"
        index=kalman.itr_sum-1
        real_trans.transform.translation.x = kalman.X1_np[0,index]
        real_trans.transform.translation.y =kalman.X1_np[1,index]
        real_trans.transform.translation.z = kalman.X1_np[2,index]
        real_trans.transform.rotation.x =kalman.X1_np[3,index]
        real_trans.transform.rotation.y =kalman.X1_np[4,index]
        real_trans.transform.rotation.z =kalman.X1_np[5,index]
        real_trans.transform.rotation.w=kalman.X1_np[6,index]
        self.br.sendTransform(real_trans)

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

    def get_insert_trajectory(self, all_info):
        tool_len = 0.5
        trajectory =  [] 
        total_dist = 0.1
        insert_time=5
        scale_dist = total_dist/insert_time
        print('get_insert_trajectory')
        for i in range(insert_time):

            temp_dist = scale_dist * (i+1)

            # SJTU HERE CHANGED ori: z x y
            tgt_pose_in_real_frame = geometry_msgs.msg.Pose()
            tgt_pose_in_real_frame.position.x = 0
            tgt_pose_in_real_frame.position.y = 0
            tgt_pose_in_real_frame.position.z = -tool_len+temp_dist
            q = tf.transformations.quaternion_from_euler(0, 0, 0)
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
    
    def action(self, all_info, pre_result_dict,kalman,yolo):
        for param in self.action_params:
            if not param in all_info.keys():
                print(param, 'must give')
                return False
        print("param satified, start to insert")

        if not kalman.X1_np is None:
            print('real bolt detected')            
            self.add_real_frame(kalman, all_info)
            ee_pose = self.get_tgt_pose_in_world_frame(all_info)
            print('start pose')
            print(ee_pose)
            (r, p, y) = tf.transformations.euler_from_quaternion([ee_pose.orientation.x, ee_pose.orientation.y, ee_pose.orientation.z, ee_pose.orientation.w])
            print(r,p,y)
            if not self.set_arm_pose(self.group, ee_pose, self.effector):
                print("insert start failed")

            trajectory = self.get_insert_trajectory(all_info)
            curr_pose = self.group.get_current_pose(self.effector).pose
            
            for ee_pose in trajectory:
                if not self.set_arm_pose(self.group, ee_pose, self.effector):
                    print("failed")
                    ee_pose = self.group.get_current_pose(self.effector).pose
            self.print_pose(ee_pose)
            kalman.finished=True
            return {'success': True}
        return {'success': False}