#!/usr/bin/env python3

import rospy
import smach

"""This state causes the robot to move to predefined locations on
the map and search for objects in that region.
"""
class LocateObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])

    def execute(self, ud):
        return super().execute(ud)