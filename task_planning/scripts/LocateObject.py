#!/usr/bin/env python3

import math
import rospy
import smach
import hsrb_interface

from visualization_msgs.msg import Marker, MarkerArray

"""This state causes the robot to move to predefined locations on
the map and search for objects in that region. It makes sure the
exact same object is not recorded multiple times.
"""
class LocateObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["succeeded", "failed", "aborted"],
                             input_keys=["objects_found_count_in", "pickup_info"],
                             output_keys=["objects_found_count_out", "pickup_info"])

        # the number of sectors checked for objects
        self._sector_count = 0

    def _rotate_base(self, robot, deg=90) -> None:
        """Rotate the robot by an amount relative to
        its current pose.
        """
        base = robot.get("omni_base")
        base.go_rel(x=0.0, y=0.0, yaw=math.radians(deg)) 

    def _rotate_head(self, robot, pan_deg=0, tilt_deg=0):
        """Rotate the robot head about pan and tilt joints
        to the specified joint angles.
        """
        whole_body = robot.get("whole_body")
        whole_body.move_to_joint_positions(
            head_pan_joint=math.radians(pan_deg),
            head_tilt_joint=math.radians(tilt_deg)
        )

    def _is_unqiue_object(self, ud, marker):
        """Checks if the passed marker belongs to an object
        which has been already identified.
        """
        # TODO: use RMS-error between marker centroid
        # and all identified objects to see if it's unique
        pass

    def look_new_sector(self, robot, move_base=True) -> None:
        """Rotate the robot and tilt its head to 
        the next sector relative to its current pose.
        """
        if move_base: 
            self._rotate_base(robot, 90)
        self._rotate_head(robot, pan_deg=0, tilt_deg=-45)

    """TODO:
    1. we will look down to search position
    2. check if objects are visible
    3. if no objects are visible then rotate by 90deg and go back to step 2. 
    4. if object/s is visible return 'succeeded'
    5. after searching 360 deg return 'failed'
    """
    def execute(self, ud):
        rospy.loginfo("Executing state LocateObject")

        status = "aborted"
        with hsrb_interface.Robot() as robot:
            self.look_new_sector(robot, move_base=False)

            # keep looping until all sectors checked or 3 unique objects found
            while True:
                # check if objects are detected
                try:
                    marker_array_msg = rospy.wait_for_message("/text_markers", 
                                                           MarkerArray, 
                                                           timeout=2.0)
                    rospy.loginfo(f"Found {len(marker_array_msg.markers)} objects in vision.")

                    # filter exactly the same objects as specified by their
                    # centroid locations 
                    for marker in marker_array_msg.markers:
                        if ud.objects_found_count_out < 3:
                            if self._is_unqiue_object(ud, marker):
                                # TODO: add the info into pickup_info
                                ud.objects_found_count_out += 1
                        else:
                            break
                except rospy.ROSException as e:
                    rospy.logwarn(f"Timeout reached. No message received within 2 seconds. 
                                  Error: {e}")

                if self._sector_count == 3 and ud.objects_found_count_in < 3:
                    # checked all sectors and < 3 objects found
                    status = "failed"
                    break
                elif ud.objects_found_count_in == 3: # TODO: could probably just use len(pickup_info)
                    # 3 objects found
                    status = "succeeded"
                    break
            
                self.look_new_sector(robot)
                self._sector_count += 1

        return status
