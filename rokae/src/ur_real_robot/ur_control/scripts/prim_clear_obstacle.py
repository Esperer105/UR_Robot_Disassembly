#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from bolt_detector import BoltDetector
from rigid_transform_3D import rigid_transform_3D
from prim_base import PrimBase
import math
import geometry_msgs.msg
import tf


class PrimClearObstacle(PrimBase):
    def get_circle_trajectory(self, all_info):
        trajectory = []
        delta_angle = 30
        scale_angle = delta_angle * math.pi / 180
        radius = 0.012
        # robot move following vector
        print('get_circle_trajectory')
        for i in range(360 / delta_angle + 1):

            tamp_angle = scale_angle * i

            x_in_bolt_frame = radius * math.cos(tamp_angle)
            y_in_bolt_frame = radius * math.sin(tamp_angle)
            z_in_bolt_frame = -0.302

            tgt_pose_in_bolt_frame = geometry_msgs.msg.Pose()
            tgt_pose_in_bolt_frame.position.x = z_in_bolt_frame
            tgt_pose_in_bolt_frame.position.y = x_in_bolt_frame
            tgt_pose_in_bolt_frame.position.z = y_in_bolt_frame
            # q = tf.transformations.quaternion_from_euler(0, 1.57, 0)
            q = tf.transformations.quaternion_from_euler(0, 1.57, 0)
            tgt_pose_in_bolt_frame.orientation.x = q[0]
            tgt_pose_in_bolt_frame.orientation.y = q[1]
            tgt_pose_in_bolt_frame.orientation.z = q[2]
            tgt_pose_in_bolt_frame.orientation.w = q[3]
            # self.print_pose(tgt_pose_in_bolt_frame, 'tgt_pose_in_bolt_frame')
            tgt_pose_in_world_frame = self.transform_pose("bolt_frame",
                                                          "base",
                                                          tgt_pose_in_bolt_frame,
                                                          all_info['timestamp'])
            if not tgt_pose_in_world_frame is None:
                # self.print_pose(tgt_pose_in_world_frame, 'get_circle_trajectory %d' % i)
                trajectory.append(tgt_pose_in_world_frame)
        if len(trajectory) > 0:
            trajectory.append(trajectory[0])
        return trajectory

    def action(self, all_info, pre_result_dict):
        for param in self.action_params:
            if not param in all_info.keys():
                print(param, 'must give')
                return False
        print("param satified, start to do clear obstacle")

        detect_ret = self.circle_detector.detect(all_info['rgb_img'],threshold=0.8)

        if 'circles' in detect_ret.keys():
            print('circle success')
            
            circles = detect_ret["circles"]
            circle = self.findBestMatchCircle(circles)

            x = circle[0]
            y = circle[1]
            self.add_bolt_frame(x, y, all_info)

            bolt_pose = self.get_bolt_pose_in_world_frame(all_info)
            trajectory = self.get_circle_trajectory(all_info)
            curr_pose = self.group.get_current_pose(self.effector).pose

            for ee_pose in trajectory:
                if not self.set_arm_pose(self.group, ee_pose, self.effector):
                    ee_pose = self.group.get_current_pose(self.effector).pose
                # self.print_pose(ee_pose)
            ee_pose = curr_pose
            if not self.set_arm_pose(self.group, ee_pose, self.effector):
                ee_pose = self.group.get_current_pose(self.effector).pose
            self.print_pose(ee_pose)
            return {'success': True, 'bolt_pose': bolt_pose}

        return {'success': False}
