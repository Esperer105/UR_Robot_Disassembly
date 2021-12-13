#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import WrenchStamped
import  moveit_commander
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import  WrenchStamped
from geometry_msgs.msg import *
import math

from actionlib import SimpleActionClient, GoalStatus

import sys, random, copy
import rospy, tf, rospkg
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import *
from std_msgs.msg import *

import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list



    
def roate_value(group1)   :
    joint_goal = group1.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -math.pi/2
    joint_goal[2] = 0
    joint_goal[3] = -math.pi/2
    joint_goal[4] = 0
    joint_goal[5] = math.pi*2
    return joint_goal

    
    
    
def MoveIt_position():
    # rospy.init_node("sort_robot_program")
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()

    group_name1 = "manipulator"
    group1 = moveit_commander.MoveGroupCommander(group_name1)
    group1.set_planner_id("RRTConnectkConfigDefault")

    group1.set_goal_joint_tolerance(0.001)
     # 设置允许的最大速度和加速度，0.5是一个比例，乘以允许的最大速度和加速度
    group1.set_max_acceleration_scaling_factor(0.05)
    group1.set_max_velocity_scaling_factor(0.05)

    group1.clear_pose_targets()
    pose_target = geometry_msgs.msg.Pose()

    roate_num=2
    joint_goal=roate_value(group1)
    print('current roate_num {0}',roate_num)
    suceed=group1.go(joint_goal, wait=False)
    print('print')
    return group1



def poseCallback(msg):

    rospy.loginfo("force : x:%0.6f, y:%0.6f, z:%0.6f " , msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z)
    
    z_force=math.fabs(  msg.wrench.force.z)
    y_force=math.fabs(  msg.wrench.force.y)
    x_force=math.fabs(  msg.wrench.force.x)

    print ('当前力矩是{0}',z_force)
    # test_motion.MoveIt_position()
    # or  y_force >1.4 or  x_force >1.4
    if  z_force>4.5  :
        print("force : x:%0.6f, y:%0.6f, z:%0.6f " , msg.wrench.force.x, msg.wrench.force.y,  msg.wrench.force.z)

        group1.stop()
        print ('程序 died')

        moveit_commander.roscpp_shutdown()
        rospy.signal_shutdown('failed')
        moveit_commander.os._exit(0)

 
def pose_subscriber():
	# ROS节点初始化
    # rospy.init_node('pose_subscriber', anonymous=True)
	# 创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数poseCallback
    rospy.Subscriber("/wrench", WrenchStamped, poseCallback)
	# 循环等待回调函数
    rospy.spin()
 



def robot_move(x,y,z,Y):
    
    group1.clear_pose_targets()
    quat=tf.transformations.quaternion_from_euler(3.14,0,Y)
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z
    pose_target.orientation.x = quat[0]
    pose_target.orientation.y = quat[1]
    pose_target.orientation.z = quat[2]
    pose_target.orientation.w = quat[3]
    group1.set_pose_target(pose_target)

    plan1 = group1.plan()
    group1.execute(plan1, wait= False)



def load_collision_battery_pack_box():

    # uodo   动态加载 碰撞盒位置
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(2)
    box_pose = geometry_msgs.msg.PoseStamped()
    scene.remove_world_object("battery_pack")
    box_pose.header.frame_id = robot.get_planning_frame()
    box_pose.pose.position.x = 0.3
    box_pose.pose.position.y =0.88
    box_pose.pose.position.z = 0.27
    box_pose.pose.orientation.w = 1.0
    # box_pose.pose.position.z = 1 # slightly above the end effector
    scene.add_box("battery_pack", box_pose, size=(3, 0.5,0.276))
    

    

def load_collision_base_box():

    # uodo   动态加载 碰撞盒位置
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(2)
    box_pose = geometry_msgs.msg.PoseStamped()
    scene.remove_world_object("box1")
    box_pose.header.frame_id = robot.get_planning_frame()
    box_pose.pose.position.x = 0
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = 0
    box_pose.pose.orientation.w = 1.0
    # box_pose.pose.position.z = 1 # slightly above the end effector
    scene.add_box("box1", box_pose, size=(3, 3, 0.01))
    
    
    
 
 
if __name__ == '__main__':
    
    
    # group1=MoveIt_position()

    rospy.init_node("sort_robot_program")
    
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    load_collision_battery_pack_box()
    load_collision_base_box()
    group_name1 = "manipulator"
    group1 = moveit_commander.MoveGroupCommander(group_name1)
    group1.set_planner_id("RRTConnectkConfigDefault")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
    scene = moveit_commander.PlanningSceneInterface()
    
    
    group1.set_goal_joint_tolerance(0.001)
     # 设置允许的最大速度和加速度，0.5是一个比例，乘以允许的最大速度和加速度
    group1.set_max_acceleration_scaling_factor(0.01)
    group1.set_max_velocity_scaling_factor(0.01)
    robot_move(0.28824,0.504720,0.66500,0)
    robot_move(0.28824,0.504720,0.65700,0)
    pose_subscriber()








# #!/usr/bin/env python
# import rospy

# from  geometry_msgs   import  *
# from std_msgs.msg import String

# from geometry_msgs.msg import Wrench

# from geometry_msgs.msg import WrenchStamped

# import geometry_msgs.msg



  
  
  
# def callback(data):
#     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
#     fake_wrench =WrenchStamped
    
    
    
# def listener():

      
#     # msg_wrench = geometry_msgs.msg.Wrench()
#     # print('x_force={0}'.format(msg_wrench.force.x))
#     # print('y_force={0}'.format(msg_wrench.force.y))
#     # print('z_force={0}'.format(msg_wrench.force.z))

#     # msg_wrenchStamped = geometry_msgs.msg.WrenchStamped()
    
#     # force = geometry_msgs.msg.Vector3()
#     # torque = geometry_msgs.msg.Vector3()
    
    
#     # In ROS, nodes are uniquely named. If two nodes with the same
#     # name are launched, the previous one is kicked off. The
#     # anonymous=True flag means that rospy will choose a unique
#     # name for our 'listener' node so that multiple listeners can
#     # run simultaneously.
#     rospy.init_node('listener', anonymous=True)

#     rospy.Subscriber('geometry_msgs/WrenchStamped', geometry_msgs.msg.WrenchStamped() , callback)

#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()

# if __name__ == '__main__':
#     listener()
