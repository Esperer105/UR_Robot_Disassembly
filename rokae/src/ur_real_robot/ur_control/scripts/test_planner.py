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
# from  bolt_position_detector
import copy
import moveit_commander


# from PIL import Image,ImageDraw
# import numpy as np
from test_aim_target import TestAimTarget
from kalman import Kalman
import tf2_ros
import geometry_msgs.msg
#  新增import
import math
import select, termios, tty


class TSTPlanner:
    def __init__(self, camera_name, rgb_topic, depth_topic, camera_info_topic):

        self.camera_name = camera_name
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.camera_info_topic = camera_info_topic
        self.bolt_trans_topic = '/NSPlanner/bolt_trans'

        #末端执行器
        self.effector= sys.argv[1] if len(sys.argv) > 1 else 'tool0'

        self.pose = None

        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

        # cv2.namedWindow("Image window", cv2.WINDOW_NORMAL)
        # cv2.setMouseCallback("Image window", self.mouse_callback)

        self.br = tf2_ros.TransformBroadcaster()

        # Have we recieved camera_info and image yet?
        self.ready_ = False

        self.bridge = CvBridge()

        self.camera_model = image_geometry.PinholeCameraModel()
        rospy.loginfo(
            'Camera {} initialised, {}, , {}'.format(self.camera_name, rgb_topic, depth_topic, camera_info_topic))
        print('')

        q = 1
        self.sub_rgb = message_filters.Subscriber(rgb_topic, Image, queue_size=q)
        self.sub_depth = message_filters.Subscriber(depth_topic, Image, queue_size=q)
        self.sub_camera_info = rospy.Subscriber(camera_info_topic, CameraInfo, self.cam_info_cb)
        self.camera_model_ready = False
        self.tss = message_filters.ApproximateTimeSynchronizer([self.sub_rgb, self.sub_depth],
                                                               queue_size=30, slop=0.2)

        self.tss.registerCallback(self.callback)


        moveit_commander.roscpp_initialize(sys.argv)
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        self.group.set_planner_id("RRTConnectkConfigDefault")
        

        #初始化stage
        self.aim_target_prim = TestAimTarget(self.group)
        self.prims = {'mate': self.aim_target_prim}
        self.action = 'disassemble'
        self.all_infos = {}
        self.ret_dict = {}
        self.bolt_pose = None
        self.all_infos_lock = threading.Lock()
        self.prim_thread = threading.Thread(target=self.do_action)
        self.prim_execution = True
        self.prim_thread.start()

    def start(self,  pose):
        if self.action != 'disassemble':
            print("Please start after previous task was done!")
            return False
        else:
            self.ret_dict['coarse_pose'] = pose
            self.action = 'start'
            self.bolt_pose = pose
            print ('start, coarse pose:')
            self.print_pose(pose)
            return True

    def do_action(self):
        #执行动作
        filter=Kalman(5)
        while self.prim_execution:
            self.action='mate'
            if self.action == 'disassemble':
                rospy.sleep(1)
                continue
            if self.all_infos_lock.acquire():
                infos = copy.deepcopy(self.all_infos)
                self.all_infos.clear()
                self.all_infos_lock.release()
                prim =  self.aim_target_prim 
                if (filter.itr_num==filter.itr_time):
                    filter.plot
                    rospy.sleep(30)
                else:
                    self.ret_dict = prim.action(infos, self.ret_dict,filter)
                    if 'bolt_pose' in self.ret_dict.keys():
                        self.bolt_pose = self.ret_dict['bolt_pose']
            rospy.sleep(1)

    def cam_info_cb(self, msg):
        self.camera_model.fromCameraInfo(msg)
        self.camera_model_ready = True
        self.sub_camera_info.unregister()

    def callback(self, rgb_msg, depth_msg):
        try:
            if not self.camera_model_ready:
                print("camera info is not ready")
                return
            img = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
            ts = rospy.Time.now()
            #rospy.loginfo('receiving image')
            if self.all_infos_lock.acquire():
                self.all_infos = {'rgb_img': img, 'depth_img': depth_img,
                                  'camera_model': self.camera_model, 'timestamp': ts}
                self.all_infos_lock.release()

        except Exception, err:
            print("exception happen in message call back:", err)

    def __del__(self):
        self.prim_execution = False
        self.prim_thread.join()
    
    def print_pose(self,pose):
        q = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        rpy = tf.transformations.euler_from_quaternion(q)
        print '%s: position (%.2f %.2f %.2f) orientation (%.2f %.2f %.2f %.2f) RPY (%.2f %.2f %.2f)' % \
            (self.effector, pose.position.x, pose.position.y, pose.position.z, \
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, \
            rpy[0], rpy[1], rpy[2])

    def reset_arm(self):
        joints = {}
        joints["elbow_joint"] = math.pi/4.
        joints["shoulder_lift_joint"] = -math.pi/2.
        joints["shoulder_pan_joint"] = math.pi/2.
        joints["wrist_1_joint"] = -math.pi/4.
        joints["wrist_2_joint"] = -math.pi/2.
        joints["wrist_3_joint"] = 0.
        self.group.set_joint_value_target(joints)
        plan = self.group.plan()
        if len(plan.joint_trajectory.points) > 0:
            self.group.execute(plan, wait=True)
            self.print_pose(self.group.get_current_pose(self.effector).pose)
            return True
        else:
            return False

if __name__ == '__main__':

    try:
        rospy.init_node('tstplanner-moveit', anonymous=True)

        planner = TSTPlanner('camera', '/camera/color/image_raw', '/camera/aligned_depth_to_color/image_raw', '/camera/color/camera_info')
        quat = tf.transformations.quaternion_from_euler(-3.14, 0, 0)
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = -0.17
        pose_target.position.y = 0.52
        pose_target.position.z = 1.00
        pose_target.orientation.x = quat[0]
        pose_target.orientation.y = quat[1]
        pose_target.orientation.z = quat[2]
        pose_target.orientation.w = quat[3]
        planner.start(pose_target)

        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        print("Shutting down")
        cv2.destroyAllWindows()