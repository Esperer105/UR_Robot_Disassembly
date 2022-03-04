#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from prim_base import PrimBase
import math
import geometry_msgs.msg
import tf
import rospy


class PrimInsert(PrimBase):
    def action(self, all_info, pre_result_dict):
        for param in self.action_params:
            if not param in all_info.keys():
                print(param, 'must give')
                return False
        print("param satified, start to do insert")
        orgin_pose = self.group.get_current_pose(self.effector).pose
        while True:
            curr_pose = self.group.get_current_pose(self.effector).pose
            tgt_pose_in_effector_frame = geometry_msgs.msg.Pose()
            tgt_pose_in_effector_frame.position.x = 0
            tgt_pose_in_effector_frame.position.y = 0
            tgt_pose_in_effector_frame.position.z = 0.01
            q = tf.transformations.quaternion_from_euler(0, 0, 0)
            tgt_pose_in_effector_frame.orientation.x = q[0]
            tgt_pose_in_effector_frame.orientation.y = q[1]
            tgt_pose_in_effector_frame.orientation.z = q[2]
            tgt_pose_in_effector_frame.orientation.w = q[3]
            tgt_pose_in_world_frame = self.transform_pose(self.effector,
                                                          "base",
                                                          tgt_pose_in_effector_frame,
                                                          rospy.Time.now())
            if not tgt_pose_in_world_frame is None:
                tgt_pose_in_effector_frame.orientation.x = curr_pose.orientation.x
                tgt_pose_in_effector_frame.orientation.y = curr_pose.orientation.y
                tgt_pose_in_effector_frame.orientation.z = curr_pose.orientation.z
                tgt_pose_in_effector_frame.orientation.w = curr_pose.orientation.w
                print('move ahead 1 cm')
                if not self.set_arm_pose(self.group, tgt_pose_in_world_frame, self.effector):
                    break
            else:
                return {'success': False}
            #return original pose for testing.
            rospy.sleep(1)
        self.set_arm_pose(self.group, orgin_pose, self.effector)
        return {'success': True}