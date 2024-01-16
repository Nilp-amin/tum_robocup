import rospy
import smach
import smach_ros

import hsrb_interface

if __name__ == "__main__":
    rospy.init_node("hsrb_cleanup_task_manager")

    # create the SMACH FSM
    sm = smach.StateMachine(outcomes=["succeeded", "aborted", "preempted"])

    # open the container
    with sm:
        # add state to the container and define the transitions
        smach.StateMachine.add()
        pass

    # use introspection to visualise the FSM
    sis = smach_ros.IntrospectionServer("hsrb_fsm_server", sm, "/SM_ROOT")
    sis.start()

    # execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()