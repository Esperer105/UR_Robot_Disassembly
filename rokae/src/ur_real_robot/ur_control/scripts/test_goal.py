
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







def goal_move_line(x,y,z): 
    step = 5.0
    waypoints = []
    robot_pose = geometry_msgs.msg.Pose()
    now_pose = group1.get_current_pose().pose
    x_step_unit = (x-now_pose.position.x)/step # target_pose - start pose / move_step
    y_step_unit= (y-now_pose.position.y)/step # target_pose - start pose / move_step
    z_step_unit = (z-now_pose.position.z)/step # target_pose - start pose / move_step
    robot_pose.orientation.x = now_pose.orientation.x
    robot_pose.orientation.y = now_pose.orientation.y
    robot_pose.orientation.z = now_pose.orientation.z
    robot_pose.orientation.w = now_pose.orientation.w

    # robot move following vector
    for i in range(1,int(step)):
        robot_pose.position.x = now_pose.position.x + x_step_unit*i
        robot_pose.position.y = now_pose.position.y + y_step_unit*i
        robot_pose.position.z = now_pose.position.z + z_step_unit*i
        waypoints.append(copy.deepcopy(robot_pose))

    # last target pose add in memory
    robot_pose.position.x = x
    robot_pose.position.y = y
    robot_pose.position.z = z
    waypoints.append(copy.deepcopy(robot_pose))

    plan, fraction = group1.compute_cartesian_path(waypoints,0.01,0.0) 
    group1.execute(plan,wait=True)
    

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
    group1.execute(plan1, wait= True)



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
    
    
    

if __name__ == "__main__":
  
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
    group1.set_max_acceleration_scaling_factor(0.05)
    group1.set_max_velocity_scaling_factor(0.05)
      
    # robot_move(0.28824,0.504720,0.89,0)
    robot_move(0.28824,0.504720,0.66500,0)
    rospy.sleep(2)
    # goal_move_line(0.34,0.44,0.88,0)
