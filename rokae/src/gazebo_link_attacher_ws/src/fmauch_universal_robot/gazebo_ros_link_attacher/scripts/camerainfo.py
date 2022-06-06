#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import message_filters
import cv2
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.msg import ImageStamped


def callback(rgb_msg, depth_msg):
    img = bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
    depth_img = bridge.imgmsg_to_cv2(depth_msg, '16UC1')
    cv2.imshow('rgb',img)
    cv2.waitKey(0)
    cv2.imshow('depth',depth_img)
    cv2.waitKey(0)


if __name__ == '__main__':
    rospy.init_node('camera_info')
    bridge = CvBridge()
    sub_rgb = message_filters.Subscriber('/gazebo/default/robot/wrist_3_link/camera color/image', ImageStamped,queue_size=1)
    sub_depth = message_filters.Subscriber('/gazebo/default/robot/wrist_3_link/camera depth/image', ImageStamped,queue_size=1)
    tss = message_filters.ApproximateTimeSynchronizer([sub_rgb, sub_depth],queue_size=30, slop=0.2)
    tss.registerCallback(callback)
    while not rospy.is_shutdown():
            rospy.spin()