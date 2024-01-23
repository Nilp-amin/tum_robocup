#!/usr/bin/env python3

import rospy
import smach
import hsrb_interface

"""This state causes the robot to move to predefined locations on
the map and search for objects in that region. It makes sure the
exact same object is not recorded multiple times.
"""
class LocateObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])

    """TODO:
    1. we will look down to search position
    2. check if objects are visible
    3. if no objects are visible then rotate by 90deg and go back to step 2. 
    4. if object/s is visible return 'succeeded'
    5. after searching 360 deg return 'failed'
    """
    def execute(self, ud):
        return super().execute(ud)