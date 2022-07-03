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
from prim_aim_target import PrimAimTarget
from prim_clear_obstacle import PrimClearObstacle
from prim_insert import PrimInsert
from prim_move import PrimMove
import tf2_ros
import geometry_msgs.msg
#  新增import
from prim_action import PrimAction
from Queue import Queue
import math
import select, termios, tty


class NSPlanner:
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
        self.stage={'have_coarse_pose':False, 'above_bolt':False,'target_aim':False, 'target_clear':False,'cramped':False,'disassembled':False}
        self.aim_target_prim = PrimAimTarget(self.group)
        self.clear_obstacle_prim = PrimClearObstacle(self.group)
        self.insert_prim = PrimInsert(self.group)
        self.move_prim = PrimMove(self.group)
        self.prims = {'mate': self.aim_target_prim,
                      'push': self.clear_obstacle_prim,
                      'insert': self.insert_prim,
                      'move': self.move_prim}
        self.action = 'disassemble'
        self.all_infos = {}
        self.ret_dict = {}
        self.bolt_pose = None
        self.all_infos_lock = threading.Lock()
        self.prim_thread = threading.Thread(target=self.do_action)
        self.prim_execution = True
        self.shut_down = False
        self.prim_thread.start()

    #规划方案
    def auto_plan(self,original_stage):
        print ('start to plan')
        ori_stage=original_stage

        #建立动作原语集
        move=PrimAction('move')
        mate=PrimAction('mate')
        push=PrimAction('push')
        insert=PrimAction('insert')
        disassemble=PrimAction('disassemble')
        prim_list=(move,mate,push,insert,disassemble)

        #基于FIFO的规划生成方法
        pathQueue=Queue(0)
        pathQueue.put([ori_stage,[]])
        plan_is_end=False
        while not plan_is_end:
            tmp_pair=pathQueue.get()
            tmp_stage=tmp_pair[0]
            tmp_path=tmp_pair[1]
            if tmp_stage['disassembled']==True:
                pathQueue.put(tmp_pair)
                plan_is_end=True
            else:
                for primi in prim_list:
                    if primi.able(tmp_stage)==True:
                        new_stage=primi.action(tmp_stage)
                        new_path=[]
                        for n in tmp_path:
                            new_path.append(n)
                        new_path.append(primi.prim)
                        pathQueue.put([new_stage,new_path])
        path_list=[]
        while not pathQueue.empty():
            path=pathQueue.get()
            path_list.append(path[1])        

        #筛选出所有最短步数的规划方案
        min_step=100
        for path in path_list:
            if len(path)<min_step:
                min_step=len(path)
        path_list=[i for i in path_list if len(i)==min_step]
        print (path_list[0])
        return path_list[0]

    def start(self,  pose):
        if self.action != 'disassemble':
            print("Please start after previous task was done!")
            return False
        else:
            self.ret_dict['coarse_pose'] = pose
            self.stage['have_coarse_pose']= True
            self.action = 'start'
            self.bolt_pose = pose
            print ('start, coarse pose:')
            self.print_pose(pose)
            return True

    def do_action(self):
        #initialize primitive set
        move=PrimAction('move')
        mate=PrimAction('mate')
        push=PrimAction('push')
        insert=PrimAction('insert')
        disassemble=PrimAction('disassemble')
        prim_dict={'move':move,'mate':mate,'push':push,'insert':insert,'disassemble':disassemble}

        while self.prim_execution:
            if self.action == 'disassemble':
                rospy.sleep(1)
                continue
            else:
                if self.action == 'start':
                    print('action==start do auto_plan')
                    step_list = self.auto_plan(self.stage)
                    i = 0
                    self.action = step_list[i]
                    print(self.action)
                if self.all_infos_lock.acquire():
                    infos = copy.deepcopy(self.all_infos)
                    self.all_infos.clear()
                    self.all_infos_lock.release()
                    if self.action in prim_dict.keys():
                        # 检测pre是否满足
                        pre_is_ok = True
                        for pre in (prim_dict[self.action]).pre:
                            if not self.stage[pre] == (prim_dict[self.action].pre)[pre]:
                                pre_is_ok = False
                                break
                        if pre_is_ok == True:
                            prim = self.prims[self.action]
                            #execute primitive
                            infos['planner_handler']=self
                            self.ret_dict = prim.action(infos, self.ret_dict)
                            if 'bolt_pose' in self.ret_dict.keys():
                                self.bolt_pose = self.ret_dict['bolt_pose']
                            #update effect
                            for eff in (prim_dict[self.action]).eff:
                                self.stage[eff] = (prim_dict[self.action].eff)[eff]
                            i = i + 1
                            self.action=step_list[i]
                        else:
                            #若pre不满足，重新生成规划方案
                            step_list=self.auto_plan(self.stage)
                            i=0
                            self.action=step_list[i]
                    rospy.sleep(1)

    # def do_action(self):
    #     #建立动作元语pre集
    #     move=PrimAction('move')
    #     mate=PrimAction('mate')
    #     push=PrimAction('push')
    #     insert=PrimAction('insert')
    #     disassemble=PrimAction('disassemble')
    #     prim_dict={'move':move,'mate':mate,'push':push,'insert':insert,'disassemble':disassemble}
    #
    #     #生成规划方案
    #     step_list=self.auto_plan(self.stage)
    #     i=0
    #
    #     #执行动作
    #     while self.prim_execution:
    #         self.action=step_list[i]
    #         if self.action == 'disassemble':
    #             rospy.sleep(1)
    #             continue
    #         if self.all_infos_lock.acquire():
    #             infos = copy.deepcopy(self.all_infos)
    #             self.all_infos.clear()
    #             self.all_infos_lock.release()
    #             if self.action in prim_dict.keys():
    #                 #检测pre是否满足
    #                 pre_is_ok=True
    #                 for pre in (prim_dict[self.action]).pre:
    #                     if not self.stage[pre]==(prim_dict[self.action].pre)[pre]:
    #                         pre_is_ok=False
    #                         break
    #                 if pre_is_ok==True:
    #                     prim = self.prims[self.action]
    #
    #                     for eff in (prim_dict[self.action]).eff:
    #                         self.stage[eff]=(prim_dict[self.action].eff)[eff]
    #                     #更新stage
    #                     self.ret_dict = prim.action(infos, self.ret_dict)
    #                     if 'bolt_pose' in self.ret_dict.keys():
    #                         self.bolt_pose = self.ret_dict['bolt_pose']
    #                     i=i+1
    #                 else:
    #                     #若pre不满足，重新生成规划方案
    #                     step_list=self.auto_plan(self.stage)
    #                     i=0
    #         input('finish one primitive')
    #         rospy.sleep(1)
    #         if self.shut_down:
    #             break
    def get_latest_infos(self):
        print('get_latest_infos')
        if self.all_infos_lock.acquire():
            infos = copy.deepcopy(self.all_infos)
            self.all_infos.clear()
            self.all_infos_lock.release()
            return infos
        else:
            return null

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
        joints["joint1"] = 0
        joints["joint2"] = 0
        joints["joint3"] = 0
        joints["joint4"] = 0
        joints["joint5"] = 0
        joints["joint6"] = 0
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
        rospy.init_node('nsplanner-moveit', anonymous=True)

        planner = NSPlanner('camera', '/camera/color/image_raw', '/camera/aligned_depth_to_color/image_raw', '/camera/color/camera_info')
 
        pose_target = geometry_msgs.msg.Pose()
        #for ILC
        # quat = tf.transformations.quaternion_from_euler(-3.14, 0, 0)
        # pose_target.position.x = 0.53
        # pose_target.position.y = 0
        # pose_target.position.z = 0.60
        # pose_target.orientation.x = quat[0]
        # pose_target.orientation.y = quat[1]
        # pose_target.orientation.z = quat[2]
        # pose_target.orientation.w = quat[3]
        #for SJTU
        quat = tf.transformations.quaternion_from_euler(-3.14, 0, 0)
        pose_target.position.x = 0.30
        pose_target.position.y = 0.50
        pose_target.position.z = 0.85
        pose_target.orientation.x = quat[0]
        pose_target.orientation.y = quat[1]
        pose_target.orientation.z = quat[2]
        pose_target.orientation.w = quat[3]
        planner.start(pose_target)

        while not rospy.is_shutdown():
            rospy.spin()
        
        del planner

    except rospy.ROSInterruptException:
        print("Shutting down")
        cv2.destroyAllWindows()
