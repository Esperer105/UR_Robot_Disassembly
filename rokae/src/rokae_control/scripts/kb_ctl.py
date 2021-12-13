#!/usr/bin/env python
# -*- coding: utf-8 -*-  
import math
import sys
import select, termios, tty
import rospy
import moveit_commander
import tf
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import message_filters
import cv2
import image_geometry
import os

usage = """
Control the position of an end effector
---------------------------
t   : robot transform to tilt
a/q : left/right in tilt 
j/l : left/right
i/k : forward/backward
p/; : up/downw
x   : reset all joints on arms to zero
e/r : raw -/+
d/f : pitch -/+
c/v : yaw  -/+
s   : setting x y z R P Y
w   : capture image
<space> : print current pose
<CTRL-C>: quit
"""


class Camera():
    def __init__(self, camera_name, rgb_topic, depth_topic, camera_info_topic):

        self.camera_name = camera_name
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.camera_info_topic = camera_info_topic

        self.pose = None


        self.br = tf.TransformBroadcaster()

        # Have we recieved camera_info and image yet?
        self.ready_ = False

        self.bridge = CvBridge()

        self.camera_model = image_geometry.PinholeCameraModel()
        print(
            'Camera {} initialised, {}, , {}'.format(self.camera_name, rgb_topic, depth_topic, camera_info_topic))
        print('')

        q = 1
        self.sub_rgb = message_filters.Subscriber(rgb_topic, Image, queue_size=q)
        self.sub_depth = message_filters.Subscriber(depth_topic, Image, queue_size=q)
        self.sub_camera_info = message_filters.Subscriber(camera_info_topic, CameraInfo, queue_size=q)
        # self.tss = message_filters.ApproximateTimeSynchronizer([self.sub_rgb, self.sub_depth, self.sub_camera_info], queue_size=15, slop=0.4)
        self.tss = message_filters.ApproximateTimeSynchronizer([self.sub_rgb, self.sub_depth, self.sub_camera_info],
                                                               queue_size=30, slop=0.2)
        # self.tss = message_filters.TimeSynchronizer([sub_rgb], 10)

        self.tss.registerCallback(self.callback)
        self.capture = False
        directory = './images'
        if not os.path.exists(directory):
            os.makedirs(directory)

    def callback(self, rgb_msg, depth_msg, camera_info_msg):
        if not self.capture:
            return
        rgb_img_path = './images/rgb_img_%s.jpg'
        depth_img_path = './images/depth_img_%s.png'

        self.camera_model.fromCameraInfo(camera_info_msg)
        img = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
        print('receiving image')
        time_str = rospy.get_time()
        cv2.imwrite(rgb_img_path%(time_str), img)
        cv2.imwrite(depth_img_path%(time_str), depth_img)
        self.capture = False

    def set_capture(self):
        self.capture = True


def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def set_arm_pose(group, pose, effector):
    group.set_pose_target(pose, effector)
    plan = group.plan()
    if len(plan.joint_trajectory.points) > 0:
        group.execute(plan, wait=True)
        return True
    else:
        print 'no plan result'
        return False

def reset_arm(group):
    joints = {}
    joints["xmate_joint_1"] = 0.
    joints["xmate_joint_2"] = 0.
    joints["xmate_joint_3"] = 0.
    joints["xmate_joint_4"] = 0.
    joints["xmate_joint_5"] = 0.
    joints["xmate_joint_6"] = 0.
    group.set_joint_value_target(joints)
    plan = group.plan()
    if len(plan.joint_trajectory.points) > 0:
        group.execute(plan, wait=True)
        return True
    else:
        return False

def print_pose(pose):
    q = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    rpy = tf.transformations.euler_from_quaternion(q)
    print '%s: position (%.2f %.2f %.2f) orientation (%.2f %.2f %.2f %.2f) RPY (%.2f %.2f %.2f)' % \
        (effector, pose.position.x, pose.position.y, pose.position.z, \
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, \
        rpy[0], rpy[1], rpy[2])

if __name__=="__main__":
    effector = sys.argv[1] if len(sys.argv) > 1 else 'rokae_link7'

    settings = termios.tcgetattr(sys.stdin)
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('teleop_end_effector', anonymous=True)
    #group = moveit_commander.MoveGroupCommander("xarm6")
    group = moveit_commander.MoveGroupCommander("arm")
    group.set_planner_id("RRTConnectkConfigDefault")

    print usage
    ee_pose = group.get_current_pose(effector).pose
    print_pose(ee_pose)
    camera = Camera('camera', '/camera/color/image_raw', '/camera/depth/image_raw',
                    '/camera/color/camera_info')
    z_delta=0.01
    x_delta=0.01
    y_delta=0.01

    while(1):
            
            delta_distance_tilt=0.01
        
            key = get_key()
            #if key in moveBindings.keys():
            q = (ee_pose.orientation.x, ee_pose.orientation.y, ee_pose.orientation.z, ee_pose.orientation.w)
            rpy = tf.transformations.euler_from_quaternion(q)
            if key == ' ' :
                ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)

            elif key== 't':
                print 'location 45度到倾斜角'
                tf_angle=-math.pi+math.pi/4
                q = tf.transformations.quaternion_from_euler(tf_angle, 0.001130,-0.000277)
                ee_pose.position.x=-0.080070
                ee_pose.position.y=-0.154564
                ee_pose.position.z=1.135476

                ee_pose.orientation.x = q[0]
                ee_pose.orientation.y = q[1]
                ee_pose.orientation.z = q[2]
                ee_pose.orientation.w = q[3]
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)


            elif key== 'a':
                print '-zy,倾斜面的移动,会带动y,z的移动，采取45度倾斜角进行计算'
                z_tilt=1 /1.41*delta_distance_tilt
                y_tilt=1/1.41*delta_distance_tilt
                # z_tilt=1/delta_distance_tilt

                ee_pose.position.z -=z_tilt
                ee_pose.position.y -= y_tilt

                set_arm_pose(group, ee_pose, effector)
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)

            elif key== 'q':
                print '+zy,倾斜面的移动,会带动y,z的移动，采取45度倾斜角进行计算'
                z_tilt=1 /1.41*delta_distance_tilt
                y_tilt=1/1.41*delta_distance_tilt
                # z_tilt=1/delta_distance_tilt
                ee_pose.position.z +=z_tilt
                ee_pose.position.y += y_tilt
                set_arm_pose(group, ee_pose, effector)
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)


            elif key== 'c':
                print 'Y-'
                q = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2]-0.2)
                ee_pose.orientation.x = q[0]
                ee_pose.orientation.y = q[1]
                ee_pose.orientation.z = q[2]
                ee_pose.orientation.w = q[3]
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'v':
                print 'Y+'
                q = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2]+0.2)
                ee_pose.orientation.x = q[0]
                ee_pose.orientation.y = q[1]
                ee_pose.orientation.z = q[2]
                ee_pose.orientation.w = q[3]
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'd':
                print 'P-'
                q = tf.transformations.quaternion_from_euler(rpy[0], rpy[1]-0.2, rpy[2])
                ee_pose.orientation.x = q[0]
                ee_pose.orientation.y = q[1]
                ee_pose.orientation.z = q[2]
                ee_pose.orientation.w = q[3]
                print_pose(ee_pose)
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'f':
                print 'P+'
                q = tf.transformations.quaternion_from_euler(rpy[0], rpy[1]+0.2, rpy[2])
                ee_pose.orientation.x = q[0]
                ee_pose.orientation.y = q[1]
                ee_pose.orientation.z = q[2]
                ee_pose.orientation.w = q[3]
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'e':
                print 'R-'
                q = tf.transformations.quaternion_from_euler(rpy[0]-0.2, rpy[1], rpy[2])
                ee_pose.orientation.x = q[0]
                ee_pose.orientation.y = q[1]
                ee_pose.orientation.z = q[2]
                ee_pose.orientation.w = q[3]
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'r':
                print 'R+'
                q = tf.transformations.quaternion_from_euler(rpy[0]+0.2, rpy[1], rpy[2])
                ee_pose.orientation.x = q[0]
                ee_pose.orientation.y = q[1]
                ee_pose.orientation.z = q[2]
                ee_pose.orientation.w = q[3]
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'p':
                print 'z+'
                ee_pose.position.z += z_delta
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== ';':
                print 'z-'
                ee_pose.position.z -= z_delta
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'l':
                print 'y-'
                ee_pose.position.y -=y_delta
                set_arm_pose(group, ee_pose, effector)
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'j':
                print 'y+'
                ee_pose.position.y += y_delta
                set_arm_pose(group, ee_pose, effector)
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'i':
                print 'x+'
                ee_pose.position.x +=x_delta
                set_arm_pose(group, ee_pose, effector)
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'k':
                print 'x-'
                ee_pose.position.x -=x_delta
                set_arm_pose(group, ee_pose, effector)
                if not set_arm_pose(group, ee_pose, effector):
                    ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key== 'x':
                print 'reset'
                reset_arm(group)
                ee_pose = group.get_current_pose(effector).pose
                print_pose(ee_pose)
            elif key == 's':
                input_str = raw_input("please input x y z R P Y:")
                val_str = input_str.split()
                if len(val_str) <> 6:
                    print 'incorrect input'
                else:
                    ee_pose.position.x = float(val_str[0])
                    ee_pose.position.y = float(val_str[1])
                    ee_pose.position.z = float(val_str[2])
                    q = tf.transformations.quaternion_from_euler(float(val_str[3]),
                                                                 float(val_str[4]),
                                                                 float(val_str[5]))
                    ee_pose.orientation.x = q[0]
                    ee_pose.orientation.y = q[1]
                    ee_pose.orientation.z = q[2]
                    ee_pose.orientation.w = q[3]
                    if not set_arm_pose(group, ee_pose, effector):
                        ee_pose = group.get_current_pose(effector).pose
                    print_pose(ee_pose)
            elif key == 'w':
                camera.set_capture()
            elif (key == '\x03'):
                break

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
