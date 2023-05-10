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

class TestMove(TestBase):
    def  __init__(self, group_):
        super(TestMove, self).__init__(group_)
        self.wrench=np.array([[0,0,0,0,0,0]])
        self.cur_wrench=np.array([[0,0,0,0,0,0]])
        self.ori_wrench=np.array([[0,0,0,0,0,0]])
        self.collect=False

    def get_search_trajectory(self,attempt,radius,angle,rotation=0):
        trajectory =  [] 
        scale_radius = radius
        scale_angle = angle * math.pi / 180
        print('Generate search trajectory')
        count=0
        for i in range (int(attempt)):
            temp_radius=scale_radius*(i+1)
            for j in range( int ( (i+1)*360/angle ) ):
                temp_angle = scale_angle * j / (i+1)
                tgt_pose_in_effector_frame = geometry_msgs.msg.Pose()
                tgt_pose_in_effector_frame.position.x = temp_radius * math.cos(temp_angle)
                tgt_pose_in_effector_frame.position.y = temp_radius * math.sin(temp_angle)
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
                count+=1  
                if not tgt_pose_in_world_frame is None:
                    trajectory.append(tgt_pose_in_world_frame)
                    # print ("the %d-th trajectory"%(count))  
        if (len(trajectory) > 0):
            print ("Search trajectory generated")
            print ("total %d trajectory points"%(count))
        return trajectory
    

    def force_callback(self,msg): 
        try:
            x_force=msg.wrench.force.x
            y_force=msg.wrench.force.y
            z_force=msg.wrench.force.z
            x_torque=msg.wrench.torque.x
            y_torque=msg.wrench.torque.y
            z_torque=msg.wrench.torque.z            
            self.cur_wrench=np.array([[x_force,y_force,z_force,x_torque,y_torque,z_torque]])
            if (self.collect==True):
                if (self.wrench.all()==0):
                    self.wrench = self.cur_wrench
                else:
                    self.wrench = np.concatenate((self.wrench,self.cur_wrench), axis=0)
        except Exception as err:
            print("Exception happen in message call back:", err)
    

    def print_wrench(self):
        temp_wrench=self.cur_wrench
        if (self.ori_wrench.all()==0):
            self.ori_wrench=np.copy(temp_wrench)
            print("ori wrench: x-force:%.4f, y-force:%.4f, z-force:%.4f, x-torque:%.4f, y-torque:%.4f, z-torque:%.4f"%(temp_wrench[0,0], temp_wrench[0,1],temp_wrench[0,2],temp_wrench[0,3],temp_wrench[0,4],temp_wrench[0,5]))    
        else:
            # net_wrench=temp_wrench-self.ori_wrench
            net_wrench=temp_wrench
            print("net wrench: x-force:%.4f, y-force:%.4f, z-force:%.4f, x-torque:%.4f, y-torque:%.4f, z-torque:%.4f"%(net_wrench[0,0], net_wrench[0,1],net_wrench[0,2],net_wrench[0,3],net_wrench[0,4],net_wrench[0,5]))    


    def get_tgt_pose_in_world_frame(self,all_info,dis_range,pose_range):
        tgt_pose_in_real_frame = geometry_msgs.msg.Pose()
        tgt_pose_in_real_frame.position.x = -self.x_shift + dis_range*(random.random()-0.5)
        tgt_pose_in_real_frame.position.y = -self.y_shift + dis_range*(random.random()-0.5)
        tgt_pose_in_real_frame.position.z = -self.z_shift + dis_range*(random.random()-0.5)
        # tgt_pose_in_real_frame.position.x = -self.x_shift
        # tgt_pose_in_real_frame.position.y = -self.y_shift 
        # tgt_pose_in_real_frame.position.z = -self.z_shift 

        q = tf.transformations.quaternion_from_euler(pose_range*(random.random()-0.5), pose_range*(random.random()-0.5), pose_range*(random.random()-0.5))
        # q = tf.transformations.quaternion_from_euler(0,0,0)
        tgt_pose_in_real_frame.orientation.x = q[0]
        tgt_pose_in_real_frame.orientation.y = q[1]
        tgt_pose_in_real_frame.orientation.z = q[2]
        tgt_pose_in_real_frame.orientation.w = q[3]
        tgt_pose_in_world_frame = self.transform_pose("real_bolt_frame",
                            "base_link",
                            tgt_pose_in_real_frame,
                            all_info['bolt_ts'])
        print("start pose")
        print (tgt_pose_in_world_frame)
        (r, p, y) = tf.transformations.euler_from_quaternion([tgt_pose_in_world_frame.orientation.x, tgt_pose_in_world_frame.orientation.y, tgt_pose_in_world_frame.orientation.z, tgt_pose_in_world_frame.orientation.w])
        print(r,p,y)
        return tgt_pose_in_world_frame
    
    def get_wait_pose_in_world_frame(self,all_info):
        wait_pose_in_real_frame = geometry_msgs.msg.Pose()
        wait_pose_in_real_frame.position.x = -self.x_shift
        wait_pose_in_real_frame.position.y = -self.y_shift 
        wait_pose_in_real_frame.position.z = -self.z_shift-0.015

        q = tf.transformations.quaternion_from_euler(0,0,0)
        wait_pose_in_real_frame.orientation.x = q[0]
        wait_pose_in_real_frame.orientation.y = q[1]
        wait_pose_in_real_frame.orientation.z = q[2]
        wait_pose_in_real_frame.orientation.w = q[3]
        wait_pose_in_world_frame = self.transform_pose("real_bolt_frame",
                            "base_link",
                            wait_pose_in_real_frame,
                            all_info['bolt_ts'])
        print("wait pose")
        print (wait_pose_in_world_frame)
        (r, p, y) = tf.transformations.euler_from_quaternion([wait_pose_in_world_frame.orientation.x, wait_pose_in_world_frame.orientation.y, wait_pose_in_world_frame.orientation.z, wait_pose_in_world_frame.orientation.w])
        print(r,p,y)
        return wait_pose_in_world_frame


    def plot(self):
        fig = plt.figure(figsize=(16,9)  , dpi=120)
        ax1 = fig.add_subplot(2, 3, 1)
        ax2 = fig.add_subplot(2, 3, 2)
        ax3 = fig.add_subplot(2, 3, 3)
        ax4 = fig.add_subplot(2, 3, 4)
        ax5 = fig.add_subplot(2, 3, 5)
        ax6 = fig.add_subplot(2, 3, 6)
        print('plot is working')
        print (self.wrench.shape)
        ax1.plot(self.wrench[:,0], 'b.--', label="x_force")
        ax1.legend()
        ax2.plot(self.wrench[:,1], 'b.--', label="y_force")
        ax2.legend()        
        ax3.plot(self.wrench[:,2], 'b.--', label="z_force")
        ax3.legend()
        ax4.plot(self.wrench[:,3], 'b.--', label="x_torque")
        ax4.legend() 
        ax5.plot(self.wrench[:,4], 'b.--', label="y_torque")
        ax5.legend()       
        ax6.plot(self.wrench[:,5], 'b.--', label="z_torque")
        ax6.legend()       
        plt.suptitle('Wrench of End-effector')
        plt.savefig("wrench.png")
        print(plt.show())
        print("Plot is generated")
        np.savetxt("wrench.csv",self.wrench, delimiter=",")
        print("Data is saved")
        rospy.sleep(60)


    def action(self, all_info, pre_result_dict,kalman,yolo):
        print("start to collect wrench")
        rospy.Subscriber("/ft_wrench", WrenchStamped, self.force_callback)
        rospy.sleep(0.1)
        self.print_wrench()


        real_pose = geometry_msgs.msg.Pose()
        real_pose.position.x = 0.15415 + 0.0075
        real_pose.position.y = 0.44966 - 0.006
        real_pose.position.z = 0.06507 - 0.0075
        quat = tf.transformations.quaternion_from_euler(-math.pi, 0, -0.5*math.pi)
        real_pose.orientation.x = quat[0]
        real_pose.orientation.y = quat[1]
        real_pose.orientation.z = quat[2]
        real_pose.orientation.w = quat[3]

        self.adjust_bolt_frame(real_pose,all_info)
        wait_pose=self.get_wait_pose_in_world_frame(all_info)

        # start_pose=self.get_tgt_pose_in_world_frame(all_info,0.0025,0.05)
        start_pose=self.get_tgt_pose_in_world_frame(all_info,0.0015,0.025)
        print('Start pose generated')


        self.set_arm_pose(self.group, wait_pose, self.effector)
        rospy.sleep(1)

        self.set_arm_pose(self.group, start_pose, self.effector)
        print('True start pose:')
        curr_pose= self.group.get_current_pose(self.effector).pose
        print(curr_pose)
        (r, p, y) = tf.transformations.euler_from_quaternion([curr_pose.orientation.x, curr_pose.orientation.y, curr_pose.orientation.z, curr_pose.orientation.w])
        print(r,p,y)
        self.print_wrench()


        search_trajectory=self.get_search_trajectory(6,0.0005,60)
        self.collect=True
        for ee_pose in search_trajectory:
            if not self.set_arm_pose(self.group, ee_pose, self.effector):
                print("Search failed halfway")
                ee_pose = self.group.get_current_pose(self.effector).pose 
                print(ee_pose)
                return {'success': False}
            self.print_wrench()
        self.collect=False
        self.plot()
        return {'success': True}