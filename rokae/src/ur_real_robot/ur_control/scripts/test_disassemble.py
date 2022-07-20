#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from __future__ import print_function
from test_base import TestBase
import math
import geometry_msgs.msg
import tf
import rospy
import sys
from ur_msgs.srv import *
from ur_msgs.msg import *

class TestDisassemble(TestBase):
    def __init__(self, group_):
        super(TestDisassemble, self).__init__(group_)
        self.switch=False
    
    def get_disassemble_trajectory(self):

        trajectory =  [] 
        scale_depth= 0.025
        print('get_insert_trajectory')
        for i in range(2):
            tamp_depth=scale_depth *(i+1)
            # SJTU HERE CHANGED ori: z x y
            tgt_pose_in_effector_frame = geometry_msgs.msg.Pose()
            tgt_pose_in_effector_frame.position.x = 0
            tgt_pose_in_effector_frame.position.y = 0
            tgt_pose_in_effector_frame.position.z = -tamp_depth
            q = tf.transformations.quaternion_from_euler(0, 0, 0)
            tgt_pose_in_effector_frame.orientation.x = q[0]
            tgt_pose_in_effector_frame.orientation.y = q[1]
            tgt_pose_in_effector_frame.orientation.z = q[2]
            tgt_pose_in_effector_frame.orientation.w = q[3]
            
            tgt_pose_in_world_frame = self.transform_pose(self.effector,
                                                          "base_link",
                                                          tgt_pose_in_effector_frame,
                                                          rospy.Time.now())
            print (tgt_pose_in_world_frame)
            (r, p, y) = tf.transformations.euler_from_quaternion([tgt_pose_in_world_frame.orientation.x, tgt_pose_in_world_frame.orientation.y, tgt_pose_in_world_frame.orientation.z, tgt_pose_in_world_frame.orientation.w])
            print(r,p,y)
            if not tgt_pose_in_world_frame is None:
                trajectory.append(tgt_pose_in_world_frame)
                print ("the %d-th trajectory"%(i))
        if len(trajectory) > 0:
            print ("trajectory collected")
        return trajectory
    
    def set_digital_out(self,pin, val):
        try:
            set_io(1, pin, val)
        except rospy.ServiceException as e:
            print ("Service call failed: %s"%(e))

    def callback(self, data):
        try:
            self.switch=data.digital_out_states[0].state
        except Exception, err:
            print("exception happen in message call back:", err)

    def get_tgt_pose_in_world_frame(self):
            tgt_pose_in_effector_frame = geometry_msgs.msg.Pose()
            tgt_pose_in_effector_frame.position.x = 0
            tgt_pose_in_effector_frame.position.y = 0
            tgt_pose_in_effector_frame.position.z = -0.05
            q = tf.transformations.quaternion_from_euler(0, 0, 0)
            tgt_pose_in_effector_frame.orientation.x = q[0]
            tgt_pose_in_effector_frame.orientation.y = q[1]
            tgt_pose_in_effector_frame.orientation.z = q[2]
            tgt_pose_in_effector_frame.orientation.w = q[3]
            tgt_pose_in_world_frame = self.transform_pose(self.effector,
                                                          "base_link",
                                                          tgt_pose_in_effector_frame,
                                                          rospy.Time.now())
            print (tgt_pose_in_world_frame)
            return tgt_pose_in_world_frame

    def action(self, all_info, pre_result_dict, kalman,yolo):
        for param in self.action_params:
            if not param in all_info.keys():
                print(param, 'must give')
                return False
        print("param satified, start to disassemble")
        print("testing io-interface")
        rospy.Subscriber("/ur_hardware_interface/io_states", IOStates, self.callback)
        rospy.wait_for_service('/ur_hardware_interface/set_io')
        global set_io
        set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        print("service-server has been started") 
        trajectory = self.get_disassemble_trajectory()
        curr_pose = self.group.get_current_pose(self.effector).pose
        self.set_digital_out(0, True)
        rospy.sleep(0.1)
        print(self.switch)
        for ee_pose in trajectory:
            if not self.set_arm_pose(self.group, ee_pose, self.effector):
                print("disassemble failed")
                ee_pose = self.group.get_current_pose(self.effector).pose 
                print(ee_pose)
        self.set_digital_out(0, False)
        rospy.sleep(0.1)        
        print(self.switch)
        # ee_pose=self.get_tgt_pose_in_world_frame()
        # curr_pose=self.group.get_current_pose(self.effector).pose
        # if not self.set_arm_pose(self.group, ee_pose, self.effector):
        #     print('return failed')
        #     print(curr_pose)
        return {'success': True}