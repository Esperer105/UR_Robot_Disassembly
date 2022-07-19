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


class TestClearObstacle(TestBase):
    def get_circle_trajectory(self, all_info):
        trajectory = []
        delta_angle = 30
        scale_angle = delta_angle * math.pi / 180
        #SJTU origininal=0.012
        radius = 0.015

        tool_len = 0.5

        print('get_circle_trajectory')
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

    def action(self, all_info, pre_result_dict,kalman,yolo):
        for param in self.action_params:
            if not param in all_info.keys():
                print(param, 'must give')
                return False
        print("param satified, start to do clear obstacle")
        raw_img=all_info['rgb_img']
        height=raw_img.shape[0]
        width =raw_img.shape[1]
        print(height,width)
        crop_img=raw_img
        # crop_img=raw_img[int(0.25*height):int(0.75*height),int(0.5*(width-0.5*height)):int(0.5*(width+0.5*height))]
        # crop_img=raw_img[:,int(0.5*(width-height)):int(0.5*(width+height))]
        detect_ret=yolo.finish_YOLO_detect(crop_img)

        if 'screw' in detect_ret.keys():
            print('screw success')
            
            circles = detect_ret["screw"]
            circle = self.findBestMatchCircle(circles)
            x=circle[1]
            y=circle[0]
            # x = circle[1]+int(0.5*(width-0.5*height))
            # y = circle[0]+int(0.25*height)
            # x=circle[1]+int(0.5*(width-height))
            # y=circle[0]
            self.add_bolt_frame(x, y, all_info)

            bolt_pose = self.get_bolt_pose_in_world_frame(all_info)
            real_pose=kalman.iteration(bolt_pose)
            self.adjust_bolt_frame(real_pose,all_info)
            real_bolt_pose = geometry_msgs.msg.Pose()
            real_bolt_pose.position.x = real_pose[0,0]
            real_bolt_pose.position.y = real_pose[1,0]
            real_bolt_pose.position.z = real_pose[2,0]
            real_bolt_pose.orientation.x = real_pose[3,0]
            real_bolt_pose.orientation.y = real_pose[4,0]
            real_bolt_pose.orientation.z = real_pose[5,0]
            real_bolt_pose.orientation.w= real_pose[6,0]
            print(real_bolt_pose)
            (r, p, y) = tf.transformations.euler_from_quaternion([real_bolt_pose.orientation.x, real_bolt_pose.orientation.y, real_bolt_pose.orientation.z, real_bolt_pose.orientation.w])
            print(r,p,y)
            trajectory = self.get_circle_trajectory(all_info)
            curr_pose = self.group.get_current_pose(self.effector).pose
            
            for ee_pose in trajectory:
                if not self.set_arm_pose(self.group, ee_pose, self.effector):
                    print("failed")
                    ee_pose = self.group.get_current_pose(self.effector).pose
            
            ee_pose = curr_pose
            if not self.set_arm_pose(self.group, ee_pose, self.effector):
                ee_pose = self.group.get_current_pose(self.effector).pose
            self.print_pose(ee_pose)
            return {'success': True, 'bolt_pose': bolt_pose}
            
        return {'success': False}