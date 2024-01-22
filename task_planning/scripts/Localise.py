#!/usr/bin/env python3

import rospy
import smach

"""This state localises the robot in the environment using AMCL.
"""
class Localise(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        pass

    def execute(self, ud):
        return super().execute(ud)