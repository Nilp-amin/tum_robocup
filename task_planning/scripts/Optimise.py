#!/usr/bin/env python3

import rospy
import smach

"""This state optimises the order in which objects are to be
picked up and dropped off to maximise efficiency.
"""
class Optimise(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"])

    def execute(self, ud):
        ud.target_object.add_pickup_location(ud.target_object.get_position())
        return "succeeded"