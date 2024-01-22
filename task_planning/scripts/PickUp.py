#!/usr/bin/env python3

import rospy
import smach

"""This state tries to pickup the target object using moveit.
"""
class PickUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])

    def execute(self, ud):
        return super().execute(ud)