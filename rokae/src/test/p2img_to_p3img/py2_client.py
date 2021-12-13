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

from geometry_msgs.msg import TransformStamped

# from PIL import Image,ImageDraw
# import numpy as np
import tf2_ros

import socket
import pickle
import struct


class NSPlanner:
    def __init__(self, camera_name, rgb_topic, depth_topic, camera_info_topic):

        self.camera_name = camera_name
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.camera_info_topic = camera_info_topic
        self.bolt_trans_topic = '/NSPlanner/bolt_trans'

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
        self.group = moveit_commander.MoveGroupCommander("arm")
        self.group.set_planner_id("RRTConnectkConfigDefault")


        self.prims = {}
        self.action = 'aim'
        self.all_infos = {}
        self.all_infos_lock = threading.Lock()
        self.prim_thread = threading.Thread(target=self.do_action)
        self.prim_execution = True
        self.prim_thread.start()

        ip_port = ('127.0.0.1', 5050)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(ip_port)

    def pack_image(self, frame):
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        result, frame = cv2.imencode('.jpg', frame, encode_param)
        data = pickle.dumps(frame, 0)
        size = len(data)
        packed = struct.pack(">L", size) + data
        return packed, data, size

    def get_predicate_result(self):
        data = self.sock.recv(4096)
        result = pickle.loads(data)
        return result

    def call_edge_predicate(self, all_info):
        action_params = ['rgb_img', 'depth_img', 'camera_model', 'timestamp']
        for param in action_params:
            if not param in all_info.keys():
                print(param, 'must give')
                return False
        packed, data, size = self.pack_image(all_info['rgb_img'])
        self.sock.sendall(packed)
        print("send all finished")
        print(self.get_predicate_result())
        return True

    def plan(self):
        if self.action == 'aim':
            self.action = 'clear'
        elif self.action == 'clear':
            self.action = 'insert'
        elif self.action == 'insert':
            self.action = 'aim'

    def do_action(self):
        try:
            ret_dict = {}
            while self.prim_execution:
                if self.all_infos_lock.acquire():
                    infos = copy.deepcopy(self.all_infos)
                    self.all_infos.clear()
                    self.all_infos_lock.release()
                    self.call_edge_predicate(infos)
                    #send out frame in python2
                rospy.sleep(1)
        except Exception, err:
            print("close connection")
            packed = struct.pack(">l", -1)
            self.sock.sendall(packed)
            self.sock.close()
            print("exception happen in message do_action:", err)

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
        print("delete NSPLanner")
        self.prim_execution = False
        self.prim_thread.join()

if __name__ == '__main__':

    # 加载电池包，不加载直接回车
    # testmotion.load_battery()
    # testmotion.robot_position(0, 0, 1.5)
    try:
        rospy.init_node('nsplanner_moveit', anonymous=True)

        planner = NSPlanner('camera', '/camera/color/image_raw', '/camera/depth/image_raw', '/camera/color/camera_info')

        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        print("Shutting down")
