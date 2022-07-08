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

    def action(self, all_info, pre_result_dict, kalman):
        for param in self.request_params:
            if not param in pre_result_dict.keys():
                print(param, 'must give')
                return False
        print("param satified, start to do move")
        planner = all_info['planner_handler']
        latest_infos = planner.get_latest_infos()
        target = pre_result_dict["coarse_pose"]
        kalman.finished=True
        if not self.set_arm_pose(self.group, target, self.effector):
            return {'success': False}
        return {'success': True}