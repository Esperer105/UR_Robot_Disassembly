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
from sensor_msgs.msg import JointState


# from PIL import Image,ImageDraw
# import numpy as np 
from bolt_detector import BoltDetector
from YOLO_client import YOLO_SendImg
from rigid_transform_3D import rigid_transform_3D
from kalman import Kalman
from RANSAC_plane import cal_bolt_plane
 
class TestBase(object):
    def __init__(self, group_):
        self.tf_listener = tf.TransformListener()
        self.action_params = ['rgb_img', 'depth_img', 'camera_model', 'timestamp']
        # self.circle_detector = BoltDetector()
        # self.circle_detector.train_SVM()
        # self.circle_detector =YOLO_SendImg()
        self.group = group_
        self.effector = sys.argv[1] if len(sys.argv) > 1 else 'tool0'
        self.clamp = lambda n, minn, maxn: max(min(maxn, n), minn)
        self.br = tf2_ros.TransformBroadcaster()
        self.bolt_pos_pub = rospy.Publisher('/NSPlanner/bolt_pose', geometry_msgs.msg.PoseStamped, queue_size =10 )

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


    # def get_bolt_pose_in_world_frame_v1(self, x, y, all_info):
    #     print("get_bolt_pose_in_world_frame_v1")
    #     tl_x = self.clamp(int(x) - 1, 0, all_info['depth_img'].shape[0])
    #     br_x = self.clamp(int(x) + 1, 0, all_info['depth_img'].shape[0])
    #     tl_y = self.clamp(int(y) - 1, 0, all_info['depth_img'].shape[1])
    #     br_y = self.clamp(int(y) + 1, 0, all_info['depth_img'].shape[1])

    #     print((x, y), (tl_x, tl_y, br_x, br_y))
    #     roi = all_info['depth_img'][tl_y:br_y, tl_x:br_x]
    #     depth_distance = np.median(roi)
    #     print(depth_distance)

    #     if np.isnan(depth_distance):
    #         print("null depth info")
    #         return
    #     d = depth_distance / 1000

    #     cam_model = all_info['camera_model']
    #     coord_x = (x - cam_model.cx()) * d * (1.0 / cam_model.fx())
    #     coord_y = (y - cam_model.cy()) * d * (1.0 / cam_model.fy())
    #     coord_z = d    
    #     tgt_pose_in_cam_frame = geometry_msgs.msg.Pose()
    #     tgt_pose_in_cam_frame.position.x = coord_z
    #     tgt_pose_in_cam_frame.position.y = -coord_x
    #     tgt_pose_in_cam_frame.position.z = -coord_y
    #     # q = tf.transformations.quaternion_from_euler(0, 1.57, 0)
    #     q = tf.transformations.quaternion_from_euler(0, 0, 0)
    #     tgt_pose_in_cam_frame.orientation.x = q[0]
    #     tgt_pose_in_cam_frame.orientation.y = q[1]
    #     tgt_pose_in_cam_frame.orientation.z = q[2]
    #     tgt_pose_in_cam_frame.orientation.w = q[3] 
    #     tgt_pose_in_world_frame = self.transform_pose("camera_color_frame",
    #                                                   "world",
    #                                                   tgt_pose_in_cam_frame,
    #                                                   rospy.Time.now())
    #     #assuming that the bolt is vertically downward
    #     q = tf.transformations.quaternion_from_euler(0, 1.57, 0)
    #     tgt_pose_in_world_frame.orientation.x = q[0]
    #     tgt_pose_in_world_frame.orientation.y = q[1]
    #     tgt_pose_in_world_frame.orientation.z = q[2]
    #     tgt_pose_in_world_frame.orientation.w = q[3] 
    #     ps = geometry_msgs.msg.PoseStamped()
    #     ps.header.frame_id = "world"
    #     ps.header.stamp = rospy.Time.now()
    #     ps.pose = tgt_pose_in_world_frame
    #     self.bolt_pos_pub.publish(ps)
    #     return tgt_pose_in_world_frame                   

    def get_bolt_pose_in_world_frame(self,all_info):
        print('get_bolt_pose_in_world_frame')
        tgt_pose_in_bolt_frame = geometry_msgs.msg.Pose()
        tgt_pose_in_bolt_frame.position.x = 0
        tgt_pose_in_bolt_frame.position.y = 0
        tgt_pose_in_bolt_frame.position.z = 0
        # q = tf.transformations.quaternion_from_euler(0, 1.57, 0)
        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        tgt_pose_in_bolt_frame.orientation.x = q[0]
        tgt_pose_in_bolt_frame.orientation.y = q[1]
        tgt_pose_in_bolt_frame.orientation.z = q[2]
        tgt_pose_in_bolt_frame.orientation.w = q[3]
        # self.print_pose(tgt_pose_in_bolt_frame, 'tgt_pose_in_bolt_frame')
        #SJTU
        tgt_pose_in_world_frame = self.transform_pose("bolt_frame",
                                                      "base_link",
                                                      tgt_pose_in_bolt_frame,
                                                      all_info['bolt_ts'])

        # q = tf.transformations.quaternion_from_euler(-math.pi, 0, 0.5*math.pi)
        # tgt_pose_in_world_frame.orientation.x = q[0]
        # tgt_pose_in_world_frame.orientation.y = q[1]
        # tgt_pose_in_world_frame.orientation.z = q[2]
        # tgt_pose_in_world_frame.orientation.w = q[3]

        print (tgt_pose_in_world_frame)
        (r, p, y) = tf.transformations.euler_from_quaternion([tgt_pose_in_world_frame.orientation.x, tgt_pose_in_world_frame.orientation.y, tgt_pose_in_world_frame.orientation.z, tgt_pose_in_world_frame.orientation.w])
        print(r,p,y)
        #ILC
        # tgt_pose_in_world_frame = self.transform_pose("bolt_frame",
        #                                               "world",
        #                                               tgt_pose_in_bolt_frame,
        #                                               all_info['bolt_ts'])
        # self.print_pose(tgt_pose_in_world_frame, 'tgt_pose_in_world_frame')
        return tgt_pose_in_world_frame



    def set_arm_pose(self, group, pose, effector):
        joint_states = rospy.wait_for_message("joint_states",JointState)
        joint_pose = joint_states.position
        if (joint_pose[5] > math.pi):
            joints = {}
            joints["elbow_joint"] = joint_pose[0]
            joints["shoulder_lift_joint"] = joint_pose[1]
            joints["shoulder_pan_joint"] = joint_pose[2]
            joints["wrist_1_joint"] = joint_pose[3]
            joints["wrist_2_joint"] = joint_pose[4]
            joints["wrist_3_joint"] = joint_pose[5]-2*math.pi
            group.set_joint_value_target(joints)
            plan = group.plan()
            if len(plan.joint_trajectory.points) > 0:
                group.execute(plan, wait=True)
                print('hand adjusted')
            else:
                print('no plan result')
                return False
        group.set_pose_target(pose, effector)
        plan = group.plan()
        if len(plan.joint_trajectory.points) > 0:
            group.execute(plan, wait=True)
            return True
        else:
            print('no plan result')
            return False

    def calc_transform(self, x, y, d, all_info):
        print('calc_transform x:%f, y:%f, d:%f'%(x,y,d))
        cam_model = all_info['camera_model']
        center_x = self.clamp(int(x), 0, all_info['depth_img'].shape[1])
        center_y = self.clamp(int(y), 0, all_info['depth_img'].shape[0])
        tl_x = self.clamp(int(x) - 1, 0, all_info['depth_img'].shape[1])
        br_x = self.clamp(int(x) + 1, 0, all_info['depth_img'].shape[1])
        tl_y = self.clamp(int(y) - 1, 0, all_info['depth_img'].shape[0])
        br_y = self.clamp(int(y) + 1, 0, all_info['depth_img'].shape[0])
        tr_x = br_x
        tr_y = tl_y

        assert (tl_x < br_x)
        assert (tl_y < br_y)
        coord_tl_x = (tl_x - cam_model.cx()) * d * (1.0 / cam_model.fx())
        coord_tl_y = (tl_y - cam_model.cy()) * d * (1.0 / cam_model.fy())
        coord_tl_z = float(all_info['depth_img'][tl_y, tl_x]) / 1000
        print('coord_tl_z')        
        print(coord_tl_z)

        coord_tr_x = (tr_x - cam_model.cx()) * d * (1.0 / cam_model.fx())
        coord_tr_y = (tr_y - cam_model.cy()) * d * (1.0 / cam_model.fy())
        coord_tr_z = float(all_info['depth_img'][tr_y, tr_x]) / 1000
        print('coord_tr_z')
        print(coord_tr_z)

        coord_br_x = (br_x - cam_model.cx()) * d * (1.0 / cam_model.fx())
        coord_br_y = (br_y - cam_model.cy()) * d * (1.0 / cam_model.fy())
        coord_br_z = float(all_info['depth_img'][br_y, br_x]) / 1000
        print('coord_br_z')
        print(coord_br_z)

        print('calc_transform tl_x:%d, tl_y:%d, br_x:%d, br_y:%d'%(tl_x,tl_y,br_x, br_y))
        print('calc_transform tl_x:%f, tl_y:%f, br_x:%f, br_y:%f'%(coord_tl_x,coord_tl_y,
                        coord_br_x, coord_br_y))


        coord_center_x = (center_x - cam_model.cx()) * d * (1.0 / cam_model.fx())
        coord_center_y = (center_y - cam_model.cy()) * d * (1.0 / cam_model.fy())

        print('calc_transform coord_center_x:%f, coord_center_y:%f'%(
                        coord_center_x, coord_center_y))


        bolt_point_list = np.array(
            [[coord_tl_x - coord_center_x, coord_tr_x - coord_center_x, coord_br_x - coord_center_x],
             [coord_tl_y - coord_center_y, coord_tr_y - coord_center_y, coord_br_y - coord_center_y],
             [0, 0, 0]])


        #ILC
        # camera_point_list = np.array([
        #                               [coord_tl_z, coord_tr_z, coord_br_z],
        #                               [-coord_tl_x, -coord_tr_x, -coord_br_x],
        #                               [-coord_tl_y, -coord_tr_y, -coord_br_y]
        #                               ])

        #SJTU
        camera_point_list = np.array([[coord_tl_x, coord_tr_x, coord_br_x],
                                      [coord_tl_y, coord_tr_y, coord_br_y],
                                      [coord_tl_z, coord_tr_z, coord_br_z]])



        R_quat, t = rigid_transform_3D(bolt_point_list, camera_point_list)
        return R_quat, t

    def broadcast_tf(self, R_quat, t, all_info):
        trans = geometry_msgs.msg.TransformStamped()

        trans.header.stamp = rospy.Time.now()
        all_info['bolt_ts']=trans.header.stamp
        print("broadcast_tf")
        print(trans.header.stamp)
        trans.header.frame_id = "camera_aligned_depth_to_color_frame"
        trans.child_frame_id = "bolt_frame"
        # here changed
        trans.transform.translation.x = t[0]
        trans.transform.translation.y = t[1]
        trans.transform.translation.z = t[2]

        trans.transform.rotation.x = R_quat[0]
        trans.transform.rotation.y = R_quat[1]
        trans.transform.rotation.z = R_quat[2]
        trans.transform.rotation.w = R_quat[3]

        
        q = (trans.transform.rotation.x,
             trans.transform.rotation.y,
             trans.transform.rotation.z,
             trans.transform.rotation.w)
        rpy = tf.transformations.euler_from_quaternion(q)
        print ('transform RPY (%.2f, %.2f, %.2f)'%(rpy[0],rpy[1],rpy[2]))

        self.br.sendTransform(trans)

    def transform_pose(self, src_frame, tgt_frame, pose_pt, ts):
        '''
        transform pose of give point from 'src_frame' to 'tgt_frame'
        '''
        ps_src = geometry_msgs.msg.PoseStamped()
        try:
            # print ('transform pose') 
            # print (ts)
            self.tf_listener.waitForTransform(tgt_frame, src_frame, ts, rospy.Duration(3))
            ps_src.header.frame_id = src_frame
            ps_src.header.stamp = ts
            ps_src.pose = pose_pt

            ps_tgt = self.tf_listener.transformPose(tgt_frame, ps_src)
            # print('success')
            return ps_tgt.pose
        except:
            traceback.print_exc()
            return None

    def add_bolt_frame(self, x, y, all_info):
        tl_x = self.clamp(int(x) - 1, 0, all_info['depth_img'].shape[1])
        br_x = self.clamp(int(x) + 1, 0, all_info['depth_img'].shape[1])
        tl_y = self.clamp(int(y) - 1, 0, all_info['depth_img'].shape[0])
        br_y = self.clamp(int(y) + 1, 0, all_info['depth_img'].shape[0]) 

        print((x, y), (tl_x, tl_y, br_x, br_y))
        roi = all_info['depth_img'][tl_y:br_y, tl_x:br_x]
        depth_distance = np.median(roi)
        print(depth_distance)

        if np.isnan(depth_distance):
            print("null depth info")
            return
        d = depth_distance / 1000
        print(d)
        cam_model = all_info['camera_model']
        coord_x = (x - cam_model.cx()) * d * (1.0 / cam_model.fx())
        coord_y = (y - cam_model.cy()) * d * (1.0 / cam_model.fy())
        coord_z = d
        print("coord (%f, %f, %f)" % (coord_x, coord_y, coord_z))
        R_quat, t = self.calc_transform(x, y, d, all_info)
        print(R_quat)
        print(t)
        self.broadcast_tf(R_quat, t, all_info)

    def add_bolt_frameV2(self, bbox, all_info):
        tl_x = self.clamp(bbox[0], 0, all_info['depth_img'].shape[1])
        br_x = self.clamp(bbox[2], 0, all_info['depth_img'].shape[1])
        tl_y = self.clamp(bbox[1], 0, all_info['depth_img'].shape[0])
        br_y = self.clamp(bbox[3], 0, all_info['depth_img'].shape[0])
        print('nihao', tl_x, br_x, tl_y, br_y)
        R_quat, t = cal_bolt_plane(tl_x, tl_y, br_x, br_y, all_info)
        print(R_quat)
        print(t)
        self.broadcast_tf(R_quat, t, all_info)

    def adjust_bolt_frame(self, X1_pose, all_info):
        real_trans = geometry_msgs.msg.TransformStamped()
        
        real_trans.header.stamp = rospy.Time.now()
        all_info['bolt_ts']=real_trans.header.stamp
        print("real_broadcast_tf")
        print(real_trans.header.stamp)
        real_trans.header.frame_id = "base_link"
        real_trans.child_frame_id = "real_bolt_frame"
        real_trans.transform.translation.x = X1_pose.position.x
        real_trans.transform.translation.y = X1_pose.position.y
        real_trans.transform.translation.z = X1_pose.position.z
        real_trans.transform.rotation.x =X1_pose.orientation.x
        real_trans.transform.rotation.y =X1_pose.orientation.y
        real_trans.transform.rotation.z =X1_pose.orientation.z
        real_trans.transform.rotation.w=X1_pose.orientation.w

        print (real_trans.transform)
        (r, p, y) = tf.transformations.euler_from_quaternion([real_trans.transform.rotation.x, real_trans.transform.rotation.y, real_trans.transform.rotation.z, real_trans.transform.rotation.w])
        print(r,p,y)
        # q = (trans.transform.rotation.x,
        #      trans.transform.rotation.y,
        #      trans.transform.rotation.z,
        #      trans.transform.rotation.w)
        # rpy = tf.transformations.euler_from_quaternion(q)
        # print 'transform RPY (%.2f, %.2f, %.2f)'%(rpy[0],rpy[1],rpy[2])
        self.br.sendTransform(real_trans)

    def action(self, all_info, pre_result_dict,kalman,yolo):
        raise NotImplementedError