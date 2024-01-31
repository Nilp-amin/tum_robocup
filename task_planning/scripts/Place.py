#!/usr/bin/env python3

import rospy
import smach
import hsrb_interface
import math

from object_manipulation.srv import *
from hsrb_interface import geometry
from geometry_msgs.msg import Pose

"""This state tries to place the target object using moveit.
"""
class Place(smach.State):
    REQUIRED_PLACE_COUNT = 3
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["succeeded", "finished", "failed"],
                             input_keys=["current_pickup_index"],
                             output_keys=["current_pickup_index"])

        rospy.wait_for_service("dropoff")

    def _grasp(self, robot, angle:float) -> None:
        """Moves the robot gripper to the desired angle.
        """
        gripper = robot.get("gripper", robot.Items.END_EFFECTOR)
        gripper.command(angle)

    def _move_end_effector(self, robot, goal: Pose) -> None:
        """Moves the robot end effector to the desired goal pose
        relative to the base_footprint frame.
        """
        whole_body = robot.get("whole_body")
        whole_body.move_end_effector_pose(goal,
                                          ref_frame_id="base_footprint")

    def _dropoff(self) -> bool:
        try:
            dropoff = rospy.ServiceProxy("dropoff", Dropoff)
            return dropoff.call()
        except rospy.ServiceException as e:
            rospy.logwarn(f"Service call failed: {e}")

        return None

    def open_gripper(self, robot) -> None:
        """Opens the robot gripper.
        """
        self._grasp(robot, angle=1.2)

    def close_gripper(self, robot) -> None:
        """Closes the robot gripper.
        """
        self._grasp(robot, angle=0.0)

    def move_end_effector_to_place(self, robot) -> None:
        """Moves the robot end effector to be ready to place
        an object it has grasped.
        """
        place_pose = geometry.pose(x=0.5,
                                   y=0.0,
                                   z=0.2,
                                   ei=0.0,
                                   ej=math.radians(90),
                                   ek=math.radians(180),
                                   axes="szyx") 
        self._move_end_effector(robot, place_pose)

    def move_to_go(self, robot):
        """Moves the robot arm to the go position.
        """
        whole_body = robot.get("whole_body")
        whole_body.move_to_go()


    def execute(self, ud):
        status = "failed"

        with hsrb_interface.Robot() as robot:
            self.move_end_effector_to_place(robot)
            self.open_gripper(robot)

            ud.current_pickup_index += 1
            if ud.current_pickup_index < Place.REQUIRED_PLACE_COUNT:
                status = "succeeded"
            elif ud.current_pickup_index == Place.REQUIRED_PLACE_COUNT:
                status = "finished"

            self.move_to_go(robot)
            self.close_gripper(robot)

        return status