#!/usr/bin/env python
#!coding=utf-8
 
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
 
cam0_path  = '/home/ur/Desktop/picture_data/M6_bolt/'    # 已经建立好的存储cam0 文件的目录
 
def callback(data):
    # define picture to_down' coefficient of ratio
    scaling_factor = 0.5
    global count,bridge
    count = count + 1
    if count == 1:
        count = 0
        cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
        timestr = "%.6f" %  data.header.stamp.to_sec()
              #%.6f表示小数点后带有6位，可根据精确度需要修改；
        image_name = timestr+ ".jpg" #图像命名：时间戳.jpg
        cv2.imwrite(cam0_path + image_name, cv_img)  #保存；
        print('ok')
        cv2.imshow("frame" , cv_img)
        if   cv2.waitKey(30000) == ord(' '):
            print('111')
    else:
        pass
 
def displayWebcam():
    rospy.init_node('webcam_display', anonymous=True)
 
    # make a video_object and init the video object
    global count,bridge
    count = 0
    rate = rospy.Rate(1)
    bridge = CvBridge()
    rospy.Subscriber('/camera/camera/color/image_raw', Image, callback)
    rospy.spin()
 
if __name__ == '__main__':
    displayWebcam()
 