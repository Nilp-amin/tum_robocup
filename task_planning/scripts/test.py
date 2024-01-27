#!/usr/bin/env python3

import rospy
import math
import hsrb_interface

from hsrb_interface import geometry

if __name__ == "__main__":
    rospy.init_node("test_node")
    with hsrb_interface.Robot() as robot:
        whole_body = robot.get("whole_body")
        # place_pose = geometry.pose(x=0.5,
        #                            y=0.0,
        #                            z=0.2,
        #                            ei=0.0,
        #                            ej=math.radians(90),
        #                            ek=math.radians(180),
        #                            axes="szyx") 
        # whole_body.move_end_effector_pose(place_pose,
        #                                   ref_frame_id="base_footprint")
        whole_body.move_to_joint_positions(
            head_pan_joint=math.radians(0),
            head_tilt_joint=math.radians(-20)
        )


    rospy.spin()