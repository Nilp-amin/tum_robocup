#!/usr/bin/env python3

import rospy
import smach

"""This state classifies the class of the detected object using darknet_ros.
"""
class ClassifyObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])

    def execute(self, ud):
        return super().execute(ud)