#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from cv2 import fastNlMeansDenoisingColoredMulti
from test_base import TestBase
import math
import geometry_msgs.msg
from geometry_msgs.msg import WrenchStamped
import tf
import rospy
import numpy as np
from itertools import chain
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from tf import TransformListener, transformations
# import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt

# from  bolt_position_detector
import copy
import tf2_ros
import traceback
import random

# from PIL import Image,ImageDraw
# import numpy as np 
from rigid_transform_3D import rigid_transform_3D
from kalman import  Kalman
import math

class TestInsert(TestBase):
    def __init__(self, group_):
        super(TestInsert, self).__init__(group_)
        self.wrench=np.array([0,0,0,0]).reshape([4,1])
        self.cur_wrench=np.array([0,0,0,0]).reshape([4,1])
        self.ori_wrench=np.array([0,0,0,0]).reshape([4,1])
        self.is_cramped=False
        self.near_cramped=False

    def get_insert_trajectory(self,real_pose,all_info):

        trajectory =  [] 
        radius=0.0015
        delta_angle = 30
        scale_angle = delta_angle * math.pi / 180
        scale_depth= 0.002
        total_ang=240
        tool_len = 0.42
        print('get_insert_trajectory')
        for i in range( total_ang / delta_angle + 1):
            tamp_radius=radius*(1-i*delta_angle/total_ang)
            tamp_angle = scale_angle * i
            tamp_depth=scale_depth * i
            # SJTU HERE CHANGED ori: z x y
            tgt_pose_in_real_frame = geometry_msgs.msg.Pose()
            tgt_pose_in_real_frame.position.x = -0.0075+tamp_radius * math.cos(tamp_angle)
            tgt_pose_in_real_frame.position.y =0.0035+tamp_radius * math.sin(tamp_angle)
            tgt_pose_in_real_frame.position.z = -tool_len+tamp_depth
            q = tf.transformations.quaternion_from_euler(0, 0, tamp_angle)
            tgt_pose_in_real_frame.orientation.x = q[0]
            tgt_pose_in_real_frame.orientation.y = q[1]
            tgt_pose_in_real_frame.orientation.z = q[2]
            tgt_pose_in_real_frame.orientation.w = q[3]
            
            tgt_pose_in_world_frame = self.transform_pose("real_bolt_frame",
                                                          "base_link",
                                                          tgt_pose_in_real_frame,
                                                          all_info['bolt_ts'])
            # print (tgt_pose_in_world_frame)
            # (r, p, y) = tf.transformations.euler_from_quaternion([tgt_pose_in_world_frame.orientation.x, tgt_pose_in_world_frame.orientation.y, tgt_pose_in_world_frame.orientation.z, tgt_pose_in_world_frame.orientation.w])
            # print(r,p,y)

            if not tgt_pose_in_world_frame is None:
                trajectory.append(tgt_pose_in_world_frame)
                # print ("the %d-th trajectory"%(i))  

        if len(trajectory) > 0:
            print ("trajectory collected")
        return trajectory

    def get_search_trajectory(self,attempt,radius,rotation=0):

        trajectory =  [] 
        scale_radius=radius
        delta_angle = 30
        scale_angle = delta_angle * math.pi / 180

        print('get_search_trajectory')
        for i in range (attempt):
            temp_radius=scale_radius*(i+1)
            for j in range(360/ delta_angle ):
                temp_angle = scale_angle * j
                tgt_pose_in_effector_frame = geometry_msgs.msg.Pose()
                tgt_pose_in_effector_frame.position.x = temp_radius * math.cos(temp_angle)
                tgt_pose_in_effector_frame.position.y = temp_radius * math.sin(temp_angle)
                tgt_pose_in_effector_frame.position.z = 0
                q = tf.transformations.quaternion_from_euler(0, 0, j*rotation*math.pi/180)
                tgt_pose_in_effector_frame.orientation.x = q[0]
                tgt_pose_in_effector_frame.orientation.y = q[1]
                tgt_pose_in_effector_frame.orientation.z = q[2]
                tgt_pose_in_effector_frame.orientation.w = q[3]
                tgt_pose_in_world_frame = self.transform_pose(self.effector,
                                                                "base_link",
                                                                tgt_pose_in_effector_frame,
                                                                rospy.Time.now())  
                if not tgt_pose_in_world_frame is None:
                    trajectory.append(tgt_pose_in_world_frame)
                    print ("the %d-th trajectory"%(360/ delta_angle*i+j))  
        if len(trajectory) > 0:
            print ("trajectory collected")
        return trajectory

    def test_wrench(self):
        trajectory=[]
        ori_pose= self.group.get_current_pose(self.effector).pose
        tgt_pose_in_effector_frame = geometry_msgs.msg.Pose()
        tgt_pose_in_effector_frame.position.x = 0.0015
        tgt_pose_in_effector_frame.position.y = 0
        tgt_pose_in_effector_frame.position.z = 0
        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        tgt_pose_in_effector_frame.orientation.x = q[0]
        tgt_pose_in_effector_frame.orientation.y = q[1]
        tgt_pose_in_effector_frame.orientation.z = q[2]
        tgt_pose_in_effector_frame.orientation.w = q[3]
        tgt_pose_in_world_frame = self.transform_pose(self.effector,
                                                        "base_link",
                                                        tgt_pose_in_effector_frame,
                                                        rospy.Time.now())
        trajectory.append(tgt_pose_in_world_frame)
        print('x+')
        # print (tgt_pose_in_world_frame)

        tgt_pose_in_effector_frame.position.x = -0.0015
        tgt_pose_in_world_frame = self.transform_pose(self.effector,
                                                        "base_link",
                                                        tgt_pose_in_effector_frame,
                                                        rospy.Time.now())
        trajectory.append(tgt_pose_in_world_frame)
        print('x-')
        # print (tgt_pose_in_world_frame)

        tgt_pose_in_effector_frame.position.x = 0
        tgt_pose_in_effector_frame.position.y = 0.0015
        tgt_pose_in_world_frame = self.transform_pose(self.effector,
                                                        "base_link",
                                                        tgt_pose_in_effector_frame,
                                                        rospy.Time.now())
        trajectory.append(tgt_pose_in_world_frame)
        print('y+')
        # print (tgt_pose_in_world_frame)

        tgt_pose_in_effector_frame.position.y = -0.0015
        tgt_pose_in_world_frame = self.transform_pose(self.effector,
                                                        "base_link",
                                                        tgt_pose_in_effector_frame,
                                                        rospy.Time.now())
        trajectory.append(tgt_pose_in_world_frame)
        print('y-')
        # print (tgt_pose_in_world_frame)

        for ee_pose in trajectory:
            if not self.set_arm_pose(self.group, ee_pose, self.effector):
                print("test failed")
                ee_pose = self.group.get_current_pose(self.effector).pose 
                print(ee_pose)
            self.print_wrench()
        
        for i in range(2):
            if (self.wrench[1,self.wrench.shape[1]+2*i-4] > 10) and (self.wrench[1,self.wrench.shape[1]+2*i-3] > 10):
                self.is_cramped=True
                print ('Cramped!')
                break
        for i in range (4):    
            if self.wrench[0,self.wrench.shape[1]+i-4] < -3:
                self.near_cramped=True
                print ('Near cramped!')
        if not self.set_arm_pose(self.group, ori_pose, self.effector):
                print("test recovery failed")
        print('test finished')
        return ori_pose
    


    def force_callback(self,msg): 
        try:
            ver_force=msg.wrench.force.z
            y_force=msg.wrench.force.y
            x_force=msg.wrench.force.x
            hor_force=math.sqrt(pow(x_force,2)+pow(y_force,2))
            # x_torque=msg.wrench.torque.x
            # y_torque=msg.wrench.torque.y
            # z_torque=msg.wrench.torque.z            
            self.cur_wrench=np.array([ver_force,hor_force,x_force,y_force]).reshape([4,1])
        except Exception, err:
            print("exception happen in message call back:", err)
    
    def print_wrench(self):
        temp_wrench=self.cur_wrench
        if self.ori_wrench.all()==0:
            self.ori_wrench=np.copy(temp_wrench)
            print("ori-wrench: ver-force:%.3f, hor-force:%.3f, x-force:%.4f, y-force:%.4f"%(temp_wrench[0,0], temp_wrench[1,0],temp_wrench[2,0],temp_wrench[3,0]))    
        else:
            pure_wrench=temp_wrench-self.ori_wrench
            self.wrench = np.concatenate([self.wrench, pure_wrench], axis=1)
            print("pure-wrench: ver-force:%.3f, hor-force:%.3f, x-force:%.4f, y-force:%.4f,"%(pure_wrench[0,0], pure_wrench[1,0],pure_wrench[2,0],pure_wrench[3,0]))
    
    def get_recramp_trajectory(self,vector):
        print('get_recramp_trajectory')
        scale_step=0.0105
        trajectory = []
        start_pose= self.group.get_current_pose(self.effector).pose
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
        trajectory.append(tgt_pose_in_world_frame)
        # print (tgt_pose_in_world_frame)

        tgt_pose_in_effector_frame.position.x = vector[0]*scale_step
        tgt_pose_in_effector_frame.position.y = vector[1]*scale_step
        tgt_pose_in_effector_frame.position.z = 0      
        tgt_pose_in_world_frame = self.transform_pose(self.effector,
                                                        "base_link",
                                                        tgt_pose_in_effector_frame,
                                                        rospy.Time.now())
        trajectory.append(tgt_pose_in_world_frame)
        # print (tgt_pose_in_world_frame)
        if len(trajectory) > 0:
            print ("trajectory collected")
        return trajectory        

    def get_tgt_pose_in_world_frame(self,all_info):
        tool_len = 0.42
        tgt_pose_in_real_frame = geometry_msgs.msg.Pose()
        tgt_pose_in_real_frame.position.x = -0.0075
        tgt_pose_in_real_frame.position.y = 0.0035
        tgt_pose_in_real_frame.position.z = -tool_len

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

    def plot(self):

        fig = plt.figure(figsize=(16,9)  , dpi=120)
        
        ax1 = fig.add_subplot(4, 1, 1)
        ax2 = fig.add_subplot(4, 1, 2)
        ax3 = fig.add_subplot(4, 1, 3)
        ax4 = fig.add_subplot(4, 1, 4)

        print('plot is working')
        ax1.plot(self.wrench[0, :], 'go--', label="ver_force")
        ax1.legend()
        ax2.plot(self.wrench[1, :], 'go--', label="hor_force")
        ax2.legend()        
        ax3.plot(self.wrench[2, :], 'go--', label="x_force")
        ax3.legend()
        ax4.plot(self.wrench[3, :], 'go--', label="y_force")
        ax4.legend()       
        plt.suptitle('Wrench of End-effector')
        plt.savefig("wrench.png")

        print(plt.show())

    def action(self, all_info, pre_result_dict,kalman,yolo):
        # for param in self.action_params:
        #     if not param in all_info.keys():
        #         print(param, 'must give')
        #         return False
        print("param satified, start to insert")
        
        # if not kalman.get_former_pose()  is None:
        if True:
            real_pose=kalman.get_former_pose()
            # real_pose = geometry_msgs.msg.Pose()
            # real_pose.position.x = -0.0695
            # real_pose.position.x = -0.0848
            # real_pose.position.y = 0.5038
            # real_pose.position.z = 0.2312

            # real_pose.orientation.x = 1
            # real_pose.orientation.y = 0
            # real_pose.orientation.z = 0
            # real_pose.orientation.w =0

            print('real bolt detected')
            print('real pose')
            print(real_pose)
            (r, p, y) = tf.transformations.euler_from_quaternion([real_pose.orientation.x, real_pose.orientation.y, real_pose.orientation.z, real_pose.orientation.w])
            print(r,p,y)
            self.adjust_bolt_frame(real_pose,all_info)
            ee_pose=self.get_tgt_pose_in_world_frame(all_info)
            rospy.Subscriber("/wrench", WrenchStamped, self.force_callback)
            rospy.sleep(0.1)
            curr_pose= self.group.get_current_pose(self.effector).pose
            if not self.set_arm_pose(self.group, ee_pose, self.effector):
                print("failed")
                print(curr_pose)
            self.print_wrench()
            insert_trajectory=self.get_insert_trajectory(real_pose,all_info)
            for ee_pose in insert_trajectory:
                if not self.set_arm_pose(self.group, ee_pose, self.effector):
                    print("insert failed")
                    ee_pose = self.group.get_current_pose(self.effector).pose 
                    print(ee_pose)
                # self.print_wrench()
            # rospy.sleep(30)
            temp_pose=self.test_wrench()
            while not self.is_cramped:
                if self.near_cramped:
                    search_trajectory=self.get_search_trajectory(5,0.001,5)
                    for ee_pose in search_trajectory:
                        if not self.set_arm_pose(self.group, ee_pose, self.effector):
                            print("search failed")
                        self.print_wrench()
                        pre_col=self.wrench.shape[1]-1
                        last_col=self.wrench.shape[1]-2
                        if self.wrench[0,pre_col] - self.wrench[0,last_col] > 1:
                            self.near_cramped=False
                            curr_pose=self.test_wrench()
                            if self.is_cramped:
                                print ('search success')
                                break
                            print ('lost cramp')
                            break
                else:
                    search_trajectory=self.get_search_trajectory(5,0.002)
                    for ee_pose in search_trajectory:
                        if not self.set_arm_pose(self.group, ee_pose, self.effector):
                            print("search failed")
                        self.print_wrench()
                        pre_col=self.wrench.shape[1]-1
                        if self.wrench[1,pre_col]  > 10:
                            vector_x= - self.wrench[2,pre_col]
                            vector_y= - self.wrench[3,pre_col]
                            vector_xy=math.sqrt(pow(vector_x,2)+pow(vector_y,2))
                            vector=[vector_x/vector_xy , vector_y/vector_xy]
                            print(vector)
                            recramp_trajectory=self.get_recramp_trajectory(vector)
                            for ee_pose in recramp_trajectory:
                                if not self.set_arm_pose(self.group, ee_pose, self.effector):
                                    print("recramp failed")
                            self.print_wrench()                            
                            curr_pose=self.test_wrench()
                            if self.is_cramped:
                                print ('search success')
                                break
                            if self.near_cramped:
                                print ('recramp success')
                                break
            self.plot()
            return {'success': True}
        return {'success': False}