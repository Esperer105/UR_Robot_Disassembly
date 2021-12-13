#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from prim_base import PrimBase
import math
import geometry_msgs.msg
import tf
import rospy


class PrimMove(PrimBase):
    def __init__(self, group_):
        super(PrimMove, self).__init__(group_)
        self.request_params = ['coarse_pose']

    def action(self, all_info, pre_result_dict):
        for param in self.request_params:
            if not param in pre_result_dict.keys():
                print(param, 'must give')
                return False
        print("param satified, start to do move")
        target = pre_result_dict["coarse_pose"]
        if not self.set_arm_pose(self.group, target, self.effector):
            return {'success': False}
        return {'success': True}
