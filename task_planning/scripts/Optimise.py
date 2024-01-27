#!/usr/bin/env python3

import rospy
import smach
import numpy as np
from itertools import permutations
import copy

from ObjectInfo import ObjectInfo
from geometry_msgs.msg import Quaternion, PoseStamped

"""This state optimises the order in which objects are to be
picked up and dropped off to maximise efficiency.
"""
class Optimise(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "failed"],
                             input_keys=["pickup_info"],
                             output_keys=["pickup_info"])
    # def eucledian_dist(self, A, B):
    #     distance = np.linalg.norm(B-A)
    #     return distance

    def execute(self, ud):
        # ud.target_object.add_pickup_location(ud.target_object.get_position())
        rospy.loginfo("Start of Optimisation")
                ## 3 instances of class object_info
        info_object_A = ud.pickup_info[0]#.get_position()
        info_object_B = ud.pickup_info[1]#.get_position()
        info_object_C = ud.pickup_info[2]#.get_position()
        
                ## get pose of the objects and label and save in dictionary format
                ## pose is np.array([x,y]) and label is string 
        # get pointstamp_msg for pose of the object (x,y)
        point_stamped_msgA = info_object_A.get_position()
        point_stamped_msgB = info_object_B.get_position()
        point_stamped_msgC = info_object_C.get_position()

        # get the class label [["Cup", "Bottle", "Pringles"]]
        class_objectA = info_object_A.get_class()
        class_objectB = info_object_B.get_class()
        class_objectC = info_object_C.get_class()


        A = np.array([point_stamped_msgA.point.x , point_stamped_msgA.point.y])
        B = np.array([point_stamped_msgB.point.x , point_stamped_msgB.point.y])
        C = np.array([point_stamped_msgC.point.x , point_stamped_msgC.point.y])
        
        dict_objects = {
            'object_A' : A,
            'object_B' : B,
            'object_C' : C 
        }
 
        rospy.loginfo(dict_objects)
        all_combinations_dic = list(permutations(dict_objects))
        # print(len(all_combinations_dic))
        # print(all_combinations_dic[5]) 

        ## get drop of location 
        pose_stamped_msgA = info_object_A.get_dropoff_point()
        pose_stamped_msgB = info_object_B.get_dropoff_point()
        pose_stamped_msgC = info_object_C.get_dropoff_point()

        place_drop_objectA = (pose_stamped_msgA.pose.position.x , pose_stamped_msgA.pose.position.y)
        place_drop_objectB = (pose_stamped_msgB.pose.position.x , pose_stamped_msgB.pose.position.y)
        place_drop_objectC = (pose_stamped_msgC.pose.position.x , pose_stamped_msgC.pose.position.y)

        totall_distance = 0 #### HIER IN LOOP DA ICH ES 0ELN MUSS
        list_totall_distance = []
        list_totall_path = []
        ## start position -> Robot in position search_one or search_two
        if ud.nav_goal_index == 1:
             waypoint_two = rospy.get_param("/way_points/search_two")
        #      start = (waypoint_one['search_one']['x'] , waypoint_one['search_one']['y'] )
             start = (waypoint_two['x'] , waypoint_two['y'] )
             
        elif ud.nav_goal_index == 2:
             waypoint_one = rospy.get_param("/way_points/search_one")
        #      start = (waypoint_two['search_two']['x'] , waypoint_two['search_one']['y'] )
             start = (waypoint_one['x'] , waypoint_one['y'] )
             


        for j in range (0,len(all_combinations_dic)):
                sequence_order = all_combinations_dic[j]
                print(j)
                for i in range(0,len(sequence_order)): # for A , B, C 
                # print(totall_distance)
                # print(i)
                        if i == 0:
                            if  sequence_order[i] == 'object_A':
                                print('First go to object_A, pick up and bring to place_drop_objectA')
                                start_distance = np.linalg.norm(A-start)
                                distance = np.linalg.norm(place_drop_objectA - A)
                                totall_distance += distance + start_distance
                            if  sequence_order[i] == 'object_B':
                                print('First go to object_B, pick up and bring to place_drop_objectB')
                                start_distance = np.linalg.norm(B-start)
                                distance = np.linalg.norm(place_drop_objectB - B)
                                totall_distance += distance + start_distance
                            if  sequence_order[i] == 'object_C':
                                print('First go to object_C and , pick up bring to place_drop_objectC')
                                start_distance = np.linalg.norm(C-start)
                                distance = np.linalg.norm(place_drop_objectC - C)
                                totall_distance += distance + start_distance
                        else:
                            if  sequence_order[i] == 'object_A':
                                print('Go to object_A, pick up and bring to place_drop_objectA')
                                if sequence_order[i-1] == 'object_B':
                                    distance_back_object = np.linalg.norm(A-place_drop_objectB)
                                if sequence_order[i-1] == 'object_C':
                                    distance_back_object = np.linalg.norm(A-place_drop_objectC)
                                distance = np.linalg.norm(place_drop_objectA - A)
                                totall_distance += distance + distance_back_object
                            if  sequence_order[i] == 'object_B':
                                print('Go to object_B and bring to place_drop_objectB')
                                if sequence_order[i-1] == 'object_A':
                                    distance_back_object = np.linalg.norm(B-place_drop_objectA)
                                if sequence_order[i-1] == 'object_C':
                                    distance_back_object = np.linalg.norm(B-place_drop_objectC)
                                distance = np.linalg.norm(place_drop_objectB - B)
                                totall_distance += distance + distance_back_object
                            if  sequence_order[i] == 'object_C':
                                print('Go to object_C and bring to place_drop_objectC')
                                if sequence_order[i-1] == 'object_A':
                                    distance_back_object = np.linalg.norm(C-place_drop_objectA)
                                if sequence_order[i-1] == 'object_B':
                                    distance_back_object = np.linalg.norm(C-place_drop_objectB)
                                distance = np.linalg.norm(place_drop_objectC - C)
                                totall_distance += distance + distance_back_object
                print('The totall distance for the Path', sequence_order, 'is', totall_distance)

                list_totall_distance.insert(j,totall_distance)
                list_totall_path.insert(j,sequence_order)
                totall_distance = 0

        
        # print(min_distance.index(min(min_distance)))
        min_distance = min(list_totall_distance)
        min_path = list_totall_path[list_totall_distance.index(min(list_totall_distance))]
        print('The shortest path is', min_path, 'and has a distance of', min_distance)

        #            ## 3 instances of class object_info
        # info_object_A = ud.pickup_info[0]#.get_position()
        # info_object_B = ud.pickup_info[1]#.get_position()
        # info_object_C = ud.pickup_info[2]#.get_position()
        
        for l in range(0,len(min_path)):
            if min_path[l] == 'object_A':
                ud.pickup_info[l] = info_object_A
            if min_path[l] == 'object_B':
                ud.pickup_info[l] = info_object_B
            if min_path[l] == 'object_C':
                ud.pickup_info[l] = info_object_C
        


        
        
        
        
        
        return "succeeded"