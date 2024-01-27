#!/usr/bin/env python3

import rospy
import smach
import copy

from ObjectInfo import ObjectInfo
from geometry_msgs.msg import Quaternion, PoseStamped

"""This state optimises the order in which objects are to be
picked up and dropped off to maximise efficiency.
"""
class Optimise(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["succeeded"],
                             input_keys=["pickup_info"],
                             output_keys=["pickup_info"])

    def _generate_pickup_pose(self, target_object:ObjectInfo) -> PoseStamped:
        pickup_pose = PoseStamped()
        pickup_pose.header.frame_id = "map"
        pickup_pose.header.stamp = rospy.Time.now()
        pickup_pose.pose.position = target_object.get_position().point
        pickup_pose.pose.position.x -= 0.21 * 3
        pickup_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0) 

        return pickup_pose

    def execute(self, ud):
        ud.pickup_info[0].add_pickup_location(self._generate_pickup_pose(
            copy.deepcopy(ud.pickup_info[0])
        ))
        ud.pickup_info[1].add_pickup_location(self._generate_pickup_pose(
            copy.deepcopy(ud.pickup_info[1])
        ))
        ud.pickup_info[2].add_pickup_location(self._generate_pickup_pose(
            copy.deepcopy(ud.pickup_info[2])
        ))
        return "succeeded"