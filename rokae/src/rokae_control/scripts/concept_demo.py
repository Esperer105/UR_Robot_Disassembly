#!/usr/bin/env python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, random, copy
import rospy, tf, rospkg
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import *
from std_msgs.msg import *
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def product_spawn(clear_only=False):
    # This function spawn three types parts(screw1, screw2, woodbolt) in gazebo

    rospy.wait_for_service("gazebo/spawn_urdf_model")
    spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

    rospack = rospkg.RosPack()
    part_pkg = rospack.get_path('cai_env')

    max_count = 3
    max_num = 3

    for count in range(max_count):
        for num in range(0, max_num):
            item_name = "product_{0}_{1}".format(count, num)
            delete_model(item_name)
    if clear_only:
        return
    for count in range(max_count):  # three types products spawn in gazebo
        if (count == 0):
            with open(part_pkg + '/urdf/' + 'block.urdf', "r") as wood1:
                product_xml = wood1.read()
                wood1.close()

        elif (count == 1):
            with open(part_pkg + '/urdf/' + 'screw1.urdf', "r") as screw1:
                product_xml = screw1.read()
                screw1.close()
        else:
            with open(part_pkg + '/urdf/' + 'screw2.urdf', "r") as screw2:
                product_xml = screw2.read()
                screw2.close()

        for num in range(0, max_num):
            print(num)
            x_rand = random.randrange(-20, 20) * 0.01
            y_rand = random.randrange(-20, 20) * 0.01
            R_rand = random.randrange(-314, 314) * 0.01
            P_rand = random.randrange(-314, 314) * 0.01
            Y_rand = random.randrange(-314, 314) * 0.01
            if (num == 0):
                x_rand = -0.05752
                y_rand = 0.02354
                R_rand = 0
                P_rand = 0
                Y_rand = 0
            quat = tf.transformations.quaternion_from_euler(1.57, P_rand, Y_rand)
            orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
            item_name = "product_{0}_{1}".format(count, num)
            print("Spawning model:%s, %f, %f, %f", item_name, x_rand, y_rand, 1.235)
            item_pose = Pose(Point(x=x_rand, y=y_rand, z=1.235), orient)
            spawn_model(item_name, product_xml, "", item_pose, "world")
            rospy.sleep(2)

def robot_move(group, x, y, z, R, P, Y):
    # moveit_commander.roscpp_initialize(sys.argv)
    # robot = moveit_commander.RobotCommander()

    group.clear_pose_targets()
    quat = tf.transformations.quaternion_from_euler(R, P, Y)
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z
    pose_target.orientation.x = quat[0]
    pose_target.orientation.y = quat[1]
    pose_target.orientation.z = quat[2]
    pose_target.orientation.w = quat[3]
    group.set_pose_target(pose_target)

    plan = group.plan()
    group.execute(plan, wait=True)
    print(("test %f,%f,%f,%f,%f,%f")%(x,y,z,R,P,Y))


if __name__ == "__main__":
    # product_spawn()

    # First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('robot_sorting', anonymous=True)
    # Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    # the robot:
    robot = moveit_commander.RobotCommander()
    # Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    # to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()



    group_name1 = "arm"
    group1 = moveit_commander.MoveGroupCommander(group_name1)
    group1.set_planner_id("RRTConnect")
    #product_spawn()

#target_aimed False
    robot_move(group1, -0.027262, 0.028543, 1.2400, 3.14, 0, 0)
    rospy.sleep(10)

    robot_move(group1, -0.040262, 0.027543, 1.1400, 3.14, 0, 0)
    rospy.sleep(5)

    robot_move(group1, -0.076262, 0.027543, 1.1400, 3.14, 0, 0)
    rospy.sleep(5)


    robot_move(group1, -0.057262, 0.028543, 1.2800, 3.14, 0, 0)
    rospy.sleep(5)
    robot_move(group1, -0.057262, 0.038543, 1.2800, 3.14, 0, 0)
    rospy.sleep(5)

#target_aimed True
    #robot_move(group1, -0.057262, 0.038543, 1.2700, 3.14, 0, 0)


    #clear False
    #product_spawn(True)
    robot_move(group1, -0.057262, 0.038543, 1.2700, 3.14, 0, 0)

    rospy.sleep(5)


    # robot_move(group1, -0.057262, 0.038543, 1.3700, 3.14, 0, 0)
    #
    # rospy.sleep(5)
    #
    #
    # robot_move(group1, -0.047262, 0.038543, 1.2400, 3.14, 0, 0)
    # rospy.sleep(6)
    # robot_move(group1, -0.067262, 0.038543, 1.2400, 3.14, 0, 0)


    #above the bolt 1
    #robot_move(group1, -0.057262, 0.038543, 1.1700, 3.14, 0, 0)

    #socket part

    # robot_move(group1, -0.057262, 0.038543, 1.2700, 3.14, 0, 0)
    #
    # rospy.sleep(5)
    #
    # #socketed posistion
    robot_move(group1, -0.057262, 0.038543, 1.14, 3.14, 0, 0)

