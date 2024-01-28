#!/usr/bin/env python3

import rospy
import math
import smach

import hsrb_interface

from object_manipulation.srv import *
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer

"""This state tries to pickup the target object using moveit.
"""
class PickUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["succeeded", "failed"],
                             input_keys=["pickup_info", "current_pickup_index", "current_pickup_retry_count"],
                             output_keys=["current_pickup_retry_count"])

        rospy.wait_for_service("pickup")

        # subscribers for the robot point clouds
        self._labeled_object_cloud_sub = Subscriber("/labeled_object_point_cloud", PointCloud2)
        self._camera_point_cloud_sub = Subscriber("/combined_point_cloud", PointCloud2)

        # synchronise the subscribers
        self._sync_sub = ApproximateTimeSynchronizer([self._labeled_object_cloud_sub, 
                                                      self._camera_point_cloud_sub], 
                                                      queue_size=1, 
                                                      slop=0.1)
        self._sync_sub.registerCallback(self._point_cloud_callback)

        self._labeled_cloud_msg = None
        self._camera_cloud_msg = None
        self._sync_called = False

    def _rotate_head(self, robot, pan_deg=0, tilt_deg=0):
        """Rotate the robot head about pan and tilt joints
        to the specified joint angles.
        """
        whole_body = robot.get("whole_body")
        whole_body.move_to_joint_positions(
            head_pan_joint=math.radians(pan_deg),
            head_tilt_joint=math.radians(tilt_deg)
        )

    def _pickup(self, 
                env_cloud:PointCloud2, 
                obj_cloud:PointCloud2, 
                obj_class:str, 
                obj_id:int,
                centroid:PointStamped) -> PickupResponse:
        """Calls the pickup service to pickup the 
        identified objects.
        """
        try:
            pickup = rospy.ServiceProxy("pickup", Pickup)
            return pickup.call(environment_cloud=env_cloud,
                               object_cloud=obj_cloud,
                               object_class=obj_class,
                               object_id=obj_id,
                               object_centroid=centroid)
        except rospy.ServiceException as e:
            rospy.logwarn(f"Service call failed: {e}")

        return None

    def _point_cloud_callback(self, labeled_cloud_msg, camera_cloud_msg):
        self._labeled_cloud_msg = labeled_cloud_msg
        self._camera_cloud_msg = camera_cloud_msg
        self._sync_called = True

    def _grasp(self, robot, angle:float) -> None:
        """Moves the robot gripper to the desired angle.
        """
        gripper = robot.get("gripper", robot.Items.END_EFFECTOR)
        gripper.command(angle)


    def look_down(self, robot):
        """Make the robot look down at the object.
        """
        self._rotate_head(robot, pan_deg=0, tilt_deg=-45)

    def move_to_go(self, robot):
        """Moves the robot arm to the go position.
        """
        whole_body = robot.get("whole_body")
        whole_body.move_to_go()

    def close_gripper(self, robot) -> None:
        """Closes the robot gripper.
        """
        self._grasp(robot, angle=0.0)

    def execute(self, ud):
        status = "failed"
        with hsrb_interface.Robot() as robot:
            self.close_gripper(robot)
            self.look_down(robot) # TODO: could change this to use gaze_point to be more robust
            rospy.sleep(2.0)

            # wait until the sync subscriber has been called
            while not self._sync_called:
                pass

            target_object = ud.pickup_info[ud.current_pickup_index]
            response = self._pickup(self._camera_cloud_msg,
                                    self._labeled_cloud_msg,
                                    target_object.get_class(),
                                    target_object.get_class_id(),
                                    target_object.get_position())
            if response is not None:
                if response.succeeded:
                    ud.current_pickup_retry_count = 0
                    status = "succeeded"
                else:
                    ud.current_pickup_retry_count += 1

            self.move_to_go(robot)

        return status