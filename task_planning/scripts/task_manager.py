#!/usr/bin/env python3

import rospy
import smach

import hsrb_interface

import Localise, LocateObject, ClassifyObject, Optimise,    \
       PickUp, Place

from smach_ros import (IntrospectionServer, SimpleActionState)
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion

if __name__ == "__main__":
    rospy.init_node("hsrb_cleanup_task_manager")

    # create the SMACH FSM
    sm = smach.StateMachine(outcomes=["succeeded", "aborted", "preempted"])

    # define user data for the state machine
    sm.userdata.nav_goal_index = 1
    sm.userdata.objects_found_count = 0
    sm.userdata.pickup_goal = None
    sm.userdata.object_class = None
    sm.userdata.current_pickup_index = 0
    sm.userdata.current_pickup_retry_count = 0

    # open the container
    with sm:
        # navigation callback for searching for objects
        def nav_search_cb(ud, goal):
            pass

        # navigiation callback for picking up objects
        def nav_pickup_cb(ud, goal):
            pass

        # navigiation callback for dropping off objects
        def nav_dropoff_cb(ud, goal):
            pass
        
        # add state to localise hsrb
        smach.StateMachine.add("LOCALISE", Localise(), 
                               transitions={"succeeded" : "LOCATE_OBJECT_ONE",
                                            "failed" : ""})

        # navigate to location one
        smach.StateMachine.add("NAVIGATE_TO_LOCATION_ONE", 
                               SimpleActionState("move_base",
                                                 MoveBaseAction,
                                                 goal_cb=nav_search_cb,
                                                 input_keys=["nav_goal_index"],
                                                 output_keys=["nav_goal_index"]),
                                transitions={"succeeded" : "LOCATE_OBJECT_AT_ONE",
                                             "aborted" : "",
                                             "preempted" : ""})

        # navigate to location one
        smach.StateMachine.add("NAVIGATE_TO_LOCATION_TWO", 
                               SimpleActionState("move_base",
                                                 MoveBaseAction,
                                                 goal_cb=nav_search_cb,
                                                 input_keys=["nav_goal_index"],
                                                 output_keys=["nav_goal_index"]),
                                transitions={"succeeded" : "LOCATE_OBJECT_AT_TWO",
                                             "aborted" : "",
                                             "preempted" : ""})

        # locate object at location one 
        smach.StateMachine.add("LOCATE_OBJECT_AT_ONE", LocateObject(),
                               transitions={"succeeded" : "CLASSIFY_OBJECT",
                                            "failed" : "NAVIGATE_TO_LOCATION_TWO"})

        # locate object at location two
        smach.StateMachine.add("LOCATE_OBJECT_AT_TWO", LocateObject(),
                               transitions={"succeeded" : "CLASSIFY_OBJECT",
                                            "failed" : "NAVIGATE_TO_LOCATION_ONE"})

        # classify object
        smach.StateMachine.add("CLASSIFY_OBJECT", ClassifyObject(),
                               transitions={"succeeded" : "OPTIMISE",
                                            "succeeded_one" : "LOCATE_OBJECT_AT_ONE",
                                            "succeeded_two" : "LOCATE_OBJECT_AT_TWO",
                                            "failed_one" : "LOCATE_OBJECT_AT_ONE",
                                            "failed_two" : "LOCATE_OBJECT_AT_TWO"},
                                remapping={"objects_found_count_in" : "objects_found_count",
                                          "objects_found_count_out" : "objects_found_count"})
        
        # optimise the order in which to pickup and dropoff objects
        # TODO: creates a list in pickup order and contains all information
        # about locations to try in case failure of pickup, and drop off
        # locations
        smach.StateMachine.add("OPTIMISE", Optimise(),
                               transitions={"succeeded" : "NAVIGATE_TO_PICKUP"},
                               remapping={"pickup_info" : "pickup_info"})

        # navigate to the object to be picked up
        smach.StateMachine.add("NAVIGATE_TO_PICKUP",
                               SimpleActionState("move_base",
                                                 MoveBaseAction,
                                                 goal_cb=nav_pickup_cb,
                                                 input_keys=["pickup_info",
                                                             "current_pickup_index",
                                                             "current_pickup_retry_count"],
                                                 output_keys=[""]),
                                transitions={"succeeded" : "PICKUP",
                                             "aborted" : "",
                                             "preempted" : ""})

        # pickup the object
        smach.StateMachine.add("PICKUP", PickUp(),
                               transitions={"succeeded" : "NAVIGATE_TO_DROPOFF",
                                            "failed" : "NAVIGATE_TO_PICKUP"},
                                remapping={"retry_count_in" : "current_pickup_retry_count",
                                           "retry_count_out" : "current_pickup_retry_count"})

        # navigate to the object drop off location
        smach.StateMachine.add("NAVIGATE_TO_DROPOFF",
                               SimpleActionState("move_base",
                                                 MoveBaseAction,
                                                 goal_cb=nav_dropoff_cb,
                                                 input_keys=["pickup_info",
                                                             "current_pickup_index"],
                                                 output_keys=[""]),
                                transitions={"succeeded" : "PLACE",
                                             "aborted" : "",
                                             "preempted" : ""})

        # place the object at the drop off location
        smach.StateMachine.add("PLACE", Place(),
                               transitions={"succeeded" : "NAVIGATE_TO_PICKUP",
                                            "finished" : "NAVIGATE_TO_HOME",
                                            "failed" : ""},
                                remapping={"pickup_index_in" : "current_pickup_index",
                                           "pickup_index_out" : "current_pickup_index"})

        # move back to the home position
        home_goal = MoveBaseGoal()
        home_goal.target_pose.header.frame_id = "map"
        home_goal.target_pose.pose.position = Point(x=0.0, y=0.0, z=0.0)
        home_goal.target_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        smach.StateMachine.add("NAVIGATE_TO_HOME",
                               SimpleActionState("move_base",
                                                 MoveBaseAction,
                                                 goal=home_goal),
                                transitions={"succeeded" : "succeeded"})

    # use introspection to visualise the FSM
    sis = IntrospectionServer("hsrb_fsm_server", sm, "/SM_ROOT")
    sis.start()

    # execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()