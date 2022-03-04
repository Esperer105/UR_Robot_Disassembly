#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys, random, copy
import rospy, tf, rospkg
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import *
from std_msgs.msg import *

from gazebo_msgs.srv import DeleteModel

import getopt

class ArgParserError(RuntimeError):
    def __int__(self, info_):
        self.info = info_

def delete_product(model_type='v'):
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        battery_name = ''
        if model_type == 'ho':
            battery_name = 'hole_battery'

        elif model_type == 'h':
            battery_name = 'horizontal_battery'

        elif model_type == 'v':
            battery_name = 'vertical_battery'

        elif model_type == 't':
            battery_name = 'tilt_battery'

        item_name = "{0}_product".format(battery_name)
        print(("deleting model:%s")%(item_name))

        resp_delete = delete_model(item_name)
    except rospy.ServiceException as e:
        print("Delete Model service call failed: {0}".format(e))


''' spawn the model into gazebo

    Args:
    model_type:
        ho: the battery which the bolt is in a hole
        h:  the battery which the bolt is horizontal
        v:  the battery which the bolt is vertical
        t:  the battery which the bolt is tilt
       
    Return:
        the name of spawn item 
        
'''


def spawn_product(model_type_, pos_):
    rospy.wait_for_service("gazebo/spawn_urdf_model")
    spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)

    rospack = rospkg.RosPack()

    part_pkg = rospack.get_path('battery_pack_describe')

    battery_name = ''
    if model_type_ == 'ho':
        with open(part_pkg + '/urdf/' + 'hole_battery.urdf', "r") as hole_battery:
            product_xml = hole_battery.read()
            battery_name = 'hole_battery'
            hole_battery.close()

    elif model_type_ == 'h':
        with open(part_pkg + '/urdf/' + 'h_battery.urdf', "r") as h_battery:
            product_xml = h_battery.read()
            battery_name = 'horizontal_battery'
            h_battery.close()

    elif model_type_ == 'v':
        with open(part_pkg + '/urdf/' + 'v_battery.xacro', "r") as v_battery:
            product_xml = v_battery.read()
            battery_name = 'vertical_battery'
            v_battery.close()

    elif model_type_ == 't':
        with open(part_pkg + '/urdf/' + 'tilt_battery.urdf', "r") as tilt_battery:
            product_xml = tilt_battery.read()
            battery_name = 'tilt_battery'
            tilt_battery.close()

    quat = tf.transformations.quaternion_from_euler(pos_['R'], pos_['P'], pos_['Y'])
    orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
    item_name = "{0}_product".format(battery_name)
    print(("Spawning model:%s")%(item_name))
    item_pose = Pose(Point(pos_['x'], pos_['y'], pos_['z']), orient)
    spawn_model(item_name, product_xml, "", item_pose, "table")
    rospy.sleep(0.5)
    return item_name


if __name__ == "__main__":
    action = "add"
    model_type = "v"
    pos = {'x': 0, 'y': 0, 'z': 0.12, 'R': 0, 'P': 0, 'Y': 3.14}
    try:
        opts, args = getopt.getopt(sys.argv[1:], "ha:m:p:")
        for opt, arg in opts:
            if opt == '-h':
                print('env_setup.py -a [add|del] -m [ho|h|v|t] -p x,y,z,R,P,Y')
            elif opt == '-a':
                action = arg
            elif opt == '-m':
                model_type = arg
            elif opt == '-p':
                vals = arg.split(',')
                if len(vals) != 6:
                    raise ArgParserError('lenght of pos is incorrect')
                pos['x'] = float(vals[0])
                pos['y'] = float(vals[1])
                pos['z'] = float(vals[2])
                pos['R'] = float(vals[3])
                pos['P'] = float(vals[4])
                pos['Y'] = float(vals[5])
    except (getopt.GetoptError, ArgParserError):
        print('env_setup.py -a [add|del] -m [ho|h|v|t] -p x,y,z,R,P,Y')

    if action == 'add':
        spawn_product(model_type, pos)
    else:
        delete_product(model_type)
