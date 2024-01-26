#!/usr/bin/env python3

import rospy
import smach
import hsrb_interface

"""This state tries to place the target object using moveit.
"""
class Place(smach.State):
    REQUIRED_PLACE_COUNT = 3
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["succeeded", "finished", "failed"],
                             input_keys=["current_pickup_index"],
                             output_keys=["current_pickup_index"])

    def _grasp(self, robot, angle:float):
        gripper = robot.get("gripper", robot.Items.END_EFFECTOR)
        gripper.command(angle)

    def open_gripper(self, robot):
        self._grasp(robot, angle=1.2)

    def execute(self, ud):
        status = "failed"

        with hsrb_interface.Robot() as robot:
            self.open_gripper(robot)

            ud.current_pickup_index += 1
            if ud.current_pickup_index < Place.REQUIRED_PLACE_COUNT:
                status = "succeeded"
            elif ud.current_pickup_index == Place.REQUIRED_PLACE_COUNT:
                status = "finished"

        return status