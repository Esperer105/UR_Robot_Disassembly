from trajectory_msgs.msg import *
from control_msgs.msg import *
import rospy
import actionlib
from sensor_msgs.msg import JointState
 

 #!/usr/bin/env python
import sys, random, copy
import rospy, tf, rospkg
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import *
from std_msgs.msg import *

import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list



JOINT_NAMES = ['joint1_position_controller', 'joint1_position_controller', 'joint1_position_controller',
               'joint1_position_controller', 'joint1_position_controller', 'joint1_position_controller']
 
def move():
          #goal就是我们向发送的关节运动数据，实例化为FollowJointTrajectoryGoal()类
          goal = FollowJointTrajectoryGoal()
 
          #goal当中的trajectory就是我们要操作的，其余的Header之类的不用管
          goal.trajectory = JointTrajectory()
          #goal.trajectory底下一共还有两个成员，分别是joint_names和points，先给joint_names赋值
          goal.trajectory.joint_names = JOINT_NAMES
 
          #从joint_state话题上获取当前的关节角度值，因为后续要移动关节时第一个值要为当前的角度值
          joint_states = rospy.wait_for_message("joint_states",JointState)
          joints_pos = joint_states.position
 
          #给trajectory中的第二个成员points赋值
          #points中有四个变量，positions,velocities,accelerations,effort，我们给前三个中的全部或者其中一两个赋值就行了
          goal.trajectory.points=[0]*4
          goal.trajectory.points[0]=JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6,time_from_start=rospy.Duration(0.0))
          goal.trajectory.points[1]=JointTrajectoryPoint(positions=[0.5,0,-0.5,0,0,0], velocities=[0]*6,time_from_start=rospy.Duration(1.0))
          goal.trajectory.points[2]=JointTrajectoryPoint(positions=[1,0,-1,0,0,0], velocities=[0]*6,time_from_start=rospy.Duration(2.0))
          goal.trajectory.points[3]=JointTrajectoryPoint(positions=[1.57,0,-1.57,0,0,0], velocities=[0]*6,time_from_start=rospy.Duration(3.0))
          
          #发布goal，注意这里的client还没有实例化，ros节点也没有初始化，我们在后面的程序中进行如上操作
          client.send_goal(goal)
          client.wait_for_result()
 
def pub_test():
          global client
 
          #初始化ros节点
          rospy.init_node("pub_action_test")
 
          #实例化一个action的类，命名为client，与上述client对应，话题为arm_controller/follow_joint_trajectory，消息类型为FollowJointTrajectoryAction
          client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
          print("Waiting for server...")
          #等待server
          client.wait_for_server()
          print("Connect to server")
          #执行move函数，发布action
          part_pose_collect()
          move()

def part_pose_collect():
    parts_pose=[]
    model_pose = rospy.wait_for_message("gazebo/model_states",ModelStates)
    for count in range(3):
        for num in range(5):
            product_num = model_pose.name.index("product_{0}_{1}".format(count,num))
            x = model_pose.pose[product_num].position.x
            y = model_pose.pose[product_num].position.y
            z = model_pose.pose[product_num].position.z
            X = model_pose.pose[product_num].orientation.x
            Y = model_pose.pose[product_num].orientation.y
            Z = model_pose.pose[product_num].orientation.z
            W = model_pose.pose[product_num].orientation.w
            euler=tf.transformations.euler_from_quaternion((X,Y,Z,W)) 
            parts_pose.append([x,y,z,euler[0],euler[1],euler[2]])     
    # print(parts_pose)
    return parts_pose
          
 
if __name__ == "__main__":
          pub_test()