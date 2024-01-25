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
        return "succeeded"