#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from bolt_detector import BoltDetector
from rigid_transform_3D import rigid_transform_3D
from test_base import TestBase
import math
import geometry_msgs.msg
import tf
import rospy
import cv2
import numpy as np
from PIL import Image

class TestClearObstacle(TestBase):
    def get_circle_trajectory(self,real_pose,all_info):
        trajectory = []
        delta_angle = 30
        scale_angle = delta_angle * math.pi / 180
        #SJTU origininal=0.012
        radius = 0.015

        tool_len = 0.435

        print('get_circle_trajectory')
        self.adjust_bolt_frame(real_pose, all_info)        
        for i in range(360 / delta_angle + 1):

            tamp_angle = scale_angle * i

            # SJTU HERE CHANGED ori: z x y
            tgt_pose_in_real_frame = geometry_msgs.msg.Pose()
            tgt_pose_in_real_frame.position.x = radius * math.cos(tamp_angle)
            tgt_pose_in_real_frame.position.y = radius * math.sin(tamp_angle)
            tgt_pose_in_real_frame.position.z = -tool_len

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
                                                          all_info['bolt_ts'])
            print (tgt_pose_in_world_frame)
            (r, p, y) = tf.transformations.euler_from_quaternion([tgt_pose_in_world_frame.orientation.x, tgt_pose_in_world_frame.orientation.y, tgt_pose_in_world_frame.orientation.z, tgt_pose_in_world_frame.orientation.w])
            print(r,p,y)

            if not tgt_pose_in_world_frame is None:
                # self.print_pose(tgt_pose_in_world_frame, 'get_circle_trajectory %d' % i)
                trajectory.append(tgt_pose_in_world_frame)
                print ("the %d-th trajectory"%(i))
        if len(trajectory) > 0:
            trajectory.append(trajectory[0])
            print ("trajectory collected")
        return trajectory

    def get_tgt_pose_in_world_frame(self,all_info):
        tool_len = 0.435
        tgt_pose_in_real_frame = geometry_msgs.msg.Pose()
        tgt_pose_in_real_frame.position.x = 0
        tgt_pose_in_real_frame.position.y = 0
        tgt_pose_in_real_frame.position.z = -tool_len-0.15

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
        print("param satified, start to clear obstacle")
        planner = all_info['planner_handler']
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
            if 'screw' in detect_ret.keys():
                print('screw success')

                circles = detect_ret["screw"]
                #circle = self.findBestMatchCircle(circles)

                # x = circle[1]+int(0.5*(width-0.5*height))
                # y = circle[0]+int(0.25*height)
                # x=circle[1]+int(0.5*(width-height))
                # y=circle[0]
                if (s==0):
                    circle = self.findBestMatchCircle(circles)                
                    x=circle[1]-(r_width-width)/2
                    y=circle[0]-(r_height-height)/2
                    self.add_bolt_frame(x, y, latest_infos)
                    bolt_pose = self.get_bolt_pose_in_world_frame(latest_infos)
                    real_pose=kalman.iteration(bolt_pose)
                    self.adjust_bolt_frame(real_pose,latest_infos)
                    ee_pose=self.get_tgt_pose_in_world_frame(latest_infos)
                    curr_pose= self.group.get_current_pose(self.effector).pose
                    if not self.set_arm_pose(self.group, ee_pose, self.effector):
                        print("failed")
                        print(curr_pose)
                else:
                    min_diff=100
                    for screw in circles:
                        self.add_bolt_frame(screw[1]-(r_width-width)/2, screw[0]-(r_height-height)/2, latest_infos)
                        screw_pose=self.get_bolt_pose_in_world_frame(latest_infos)
                        former_pose=kalman.get_former_pose()
                        temp_diff=math.sqrt(pow(screw_pose.position.x - former_pose.position.x ,2)+pow(screw_pose.position.y - former_pose.position.y ,2)+pow(screw_pose.position.z - former_pose.position.z ,2))            
                        if (temp_diff<min_diff):
                            min_diff=temp_diff
                            near_pose=screw_pose
                    if (min_diff < 0.01):
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
                            # curr_pose= self.group.get_current_pose(self.effector).pose
                            # print(curr_pose)
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
            trajectory = self.get_circle_trajectory(real_pose,latest_infos)
            curr_pose = self.group.get_current_pose(self.effector).pose
            for ee_pose in trajectory:
                if not self.set_arm_pose(self.group, ee_pose, self.effector):
                    print("clear failed")
                    ee_pose = self.group.get_current_pose(self.effector).pose 
                    print(ee_pose)
            if not self.set_arm_pose(self.group, curr_pose, self.effector):
                curr_pose = self.group.get_current_pose(self.effector).pose
                print(curr_pose)
            

            # latest_infos = planner.get_latest_infos()
            # self.adjust_bolt_frame(real_pose,latest_infos)
            # tgt_pose_in_real_frame = geometry_msgs.msg.Pose()
            # tgt_pose_in_real_frame.position.x = 0
            # tgt_pose_in_real_frame.position.y = 0
            # tgt_pose_in_real_frame.position.z = -tool_len
            # q = tf.transformations.quaternion_from_euler(0, 0, 0)
            # tgt_pose_in_real_frame.orientation.x = q[0]
            # tgt_pose_in_real_frame.orientation.y = q[1]
            # tgt_pose_in_real_frame.orientation.z = q[2]
            # tgt_pose_in_real_frame.orientation.w = q[3]
            # tgt_pose_in_world_frame = self.transform_pose("real_bolt_frame",
            #                                               "base_link",
            #                                               tgt_pose_in_real_frame,
            #                                         latest_infos['bolt_ts'])
            # print ('final pose')
            # print (tgt_pose_in_world_frame)
            # (r, p, y) = tf.transformations.euler_from_quaternion([tgt_pose_in_world_frame.orientation.x, tgt_pose_in_world_frame.orientation.y, tgt_pose_in_world_frame.orientation.z, tgt_pose_in_world_frame.orientation.w])
            # print(r,p,y)
            # curr_pose = self.group.get_current_pose(self.effector).pose
            # if not self.set_arm_pose(self.group, tgt_pose_in_world_frame, self.effector):
            #         print("insert failed")
            #         print(curr_pose)
            

            return {'success': True, 'bolt_pose': real_pose}            
        else:
            print ('location failed')
            return {'success': False}