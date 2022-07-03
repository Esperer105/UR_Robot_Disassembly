#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import os
import threading

from torch import int32

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
import copy
import tf2_ros
import geometry_msgs.msg
import traceback
import math
import select, termios, tty
import moveit_commander


# from PIL import Image,ImageDraw
# import numpy as np 
from bolt_detector import BoltDetector
from rigid_transform_3D import rigid_transform_3D

 
class TestBase(object):
    def __init__(self, group_):
        self.tf_listener = tf.TransformListener()
        self.action_params = ['rgb_img', 'depth_img', 'camera_model', 'timestamp']
        self.circle_detector = BoltDetector()
        self.circle_detector.train_SVM()
        self.group = group_
        self.effector = sys.argv[1] if len(sys.argv) > 1 else 'tool0'
        self.clamp = lambda n, minn, maxn: max(min(maxn, n), minn)
        self.br = tf2_ros.TransformBroadcaster()

    def print_pose(self,pose):
        q = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        rpy = tf.transformations.euler_from_quaternion(q)
        print '%s: position (%.2f %.2f %.2f) orientation (%.2f %.2f %.2f %.2f) RPY (%.2f %.2f %.2f)' % \
            (self.effector, pose.position.x, pose.position.y, pose.position.z, \
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, \
            rpy[0], rpy[1], rpy[2])

    def findBestMatchCircle(self, circles):
        assert len(circles) > 0
        return circles[0]

    def get_bolt_pose_in_world_frame(self,all_info):
        tgt_pose_in_bolt_frame = geometry_msgs.msg.Pose()
        tgt_pose_in_bolt_frame.position.x = 0
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
                                                      all_info['bolt_ts'])
        # self.print_pose(tgt_pose_in_world_frame, 'tgt_pose_in_world_frame')
        return tgt_pose_in_world_frame


    def set_arm_pose(self, group, pose, effector):
        group.set_pose_target(pose, effector)
        plan = group.plan()
        if len(plan.joint_trajectory.points) > 0:
            group.execute(plan, wait=True)
            return True
        else:
            print('no plan result')
            return False

    def calc_transform(self, x, y, d, all_info):
        cam_model = all_info['camera_model']
        center_x = self.clamp(int(x), 0, all_info['depth_img'].shape[0])
        center_y = self.clamp(int(y), 0, all_info['depth_img'].shape[1])
        tl_x = self.clamp(int(x) - 1, 0, all_info['depth_img'].shape[0])
        br_x = self.clamp(int(x) + 1, 0, all_info['depth_img'].shape[0])
        tl_y = self.clamp(int(y) - 1, 0, all_info['depth_img'].shape[1])
        br_y = self.clamp(int(y) + 1, 0, all_info['depth_img'].shape[1])
        tr_x = br_x
        tr_y = tl_y
        assert (tl_x < br_x)
        assert (tl_y < br_y)
        coord_tl_x = (tl_x - cam_model.cx()) * d * (1.0 / cam_model.fx())
        coord_tl_y = (tl_y - cam_model.cy()) * d * (1.0 / cam_model.fy())
        coord_tl_z = float(all_info['depth_img'][tl_y, tl_x]) / 1000

        coord_tr_x = (tr_x - cam_model.cx()) * d * (1.0 / cam_model.fx())
        coord_tr_y = (tr_y - cam_model.cy()) * d * (1.0 / cam_model.fy())
        coord_tr_z = float(all_info['depth_img'][tr_y, tr_x]) / 1000

        coord_br_x = (br_x - cam_model.cx()) * d * (1.0 / cam_model.fx())
        coord_br_y = (br_y - cam_model.cy()) * d * (1.0 / cam_model.fy())
        coord_br_z = float(all_info['depth_img'][br_y, br_x]) / 1000

        coord_center_x = (center_x - cam_model.cx()) * d * (1.0 / cam_model.fx())
        coord_center_y = (center_y - cam_model.cy()) * d * (1.0 / cam_model.fy())

        bolt_point_list = np.array(
            [[coord_tl_x - coord_center_x, coord_tr_x - coord_center_x, coord_br_x - coord_center_x],
             [coord_tl_y - coord_center_y, coord_tr_y - coord_center_y, coord_br_y - coord_center_y],
             [0, 0, 0]])

        camera_point_list = np.array([[coord_tl_x, coord_tr_x, coord_br_x],
                                      [coord_tl_y, coord_tr_y, coord_br_y],
                                      [coord_tl_z, coord_tr_z, coord_br_z]])

        R_quat, t = rigid_transform_3D(camera_point_list, bolt_point_list)
        return R_quat, t

    def broadcast_tf(self, R_quat, t, all_info):
        trans = geometry_msgs.msg.TransformStamped()

        trans.header.stamp = rospy.Time.now()
        all_info['bolt_ts']=trans.header.stamp
        print("broadcast_tf")
        print(trans.header.stamp)
        trans.header.frame_id = "camera_aligned_depth_to_color_frame"
        trans.child_frame_id = "bolt_frame"
        trans.transform.translation.x = t[0]
        trans.transform.translation.y = t[1]
        trans.transform.translation.z = t[2]
        trans.transform.rotation.x = R_quat[0]
        trans.transform.rotation.y = R_quat[1]
        trans.transform.rotation.z = R_quat[2]
        trans.transform.rotation.w = R_quat[3]

        # q = (trans.transform.rotation.x,
        #      trans.transform.rotation.y,
        #      trans.transform.rotation.z,
        #      trans.transform.rotation.w)
        # rpy = tf.transformations.euler_from_quaternion(q)
        # print 'transform RPY (%.2f, %.2f, %.2f)'%(rpy[0],rpy[1],rpy[2])

        self.br.sendTransform(trans)

    def transform_pose(self, src_frame, tgt_frame, pose_pt, ts):
        '''
        transform pose of give point from 'src_frame' to 'tgt_frame'
        '''
        ps_src = geometry_msgs.msg.PoseStamped()
        try:
            print ('transform pose') 
            print (ts)
            self.tf_listener.waitForTransform(tgt_frame, src_frame, ts, rospy.Duration(30))
            ps_src.header.frame_id = src_frame
            ps_src.header.stamp = ts
            ps_src.pose = pose_pt

            ps_tgt = self.tf_listener.transformPose(tgt_frame, ps_src)
            return ps_tgt.pose
        except:
            traceback.print_exc()
            return None

    def add_bolt_frame(self, x, y, all_info):
        tl_x = self.clamp(int(x) - 1, 0, all_info['depth_img'].shape[0])
        br_x = self.clamp(int(x) + 1, 0, all_info['depth_img'].shape[0])
        tl_y = self.clamp(int(y) - 1, 0, all_info['depth_img'].shape[1])
        br_y = self.clamp(int(y) + 1, 0, all_info['depth_img'].shape[1])

        print((x, y), (tl_x, tl_y, br_x, br_y))
        roi = all_info['depth_img'][tl_y:br_y, tl_x:br_x]
        depth_distance = np.median(roi)
        print(depth_distance)

        if np.isnan(depth_distance):
            print("null depth info")
            return
        d = depth_distance / 1000

        cam_model = all_info['camera_model']
        coord_x = (x - cam_model.cx()) * d * (1.0 / cam_model.fx())
        coord_y = (y - cam_model.cy()) * d * (1.0 / cam_model.fy())
        coord_z = d
        print("coord (%f, %f, %f)" % (coord_x, coord_y, coord_z))
        R_quat, t = self.calc_transform(x, y, d, all_info)
        print(R_quat)
        print(t)
        self.broadcast_tf(R_quat, t, all_info)


    def action(self, all_info, pre_result_dict,kalman):
        raise NotImplementedError