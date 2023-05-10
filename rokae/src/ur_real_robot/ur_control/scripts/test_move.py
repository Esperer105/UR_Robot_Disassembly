#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from test_base import TestBase
import math
import geometry_msgs.msg
import tf
import rospy


class TestMove(TestBase):
    def __init__(self, group_):
        super(TestMove, self).__init__(group_)
        self.request_params = ['coarse_pose']


    def action(self, all_info, pre_result_dict, kalman,yolo):
        for param in self.request_params:
            if not param in pre_result_dict.keys():
                print(param, 'must give')
                return False
        print("param satified, start to do move")
        # planner = all_info['planner_handler']
        # latest_infos = planner.get_latest_infos()
        target = pre_result_dict["coarse_pose"]
        if not self.set_arm_pose(self.group, target, self.effector):
            return {'success': False}
        # rospy.sleep(2)
        # exact_pose=self.group.get_current_pose(self.effector)
        # print(exact_pose)
        # flange_pose=self.get_flange_pose_in_world_frame()
        # tool_pose=self.get_tool_pose_in_world_frame()
        # rospy.sleep(30)

        return {'success': True}