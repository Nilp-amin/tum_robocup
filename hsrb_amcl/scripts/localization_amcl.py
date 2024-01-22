#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
import geometry_msgs.msg

global loc_acc
global pose

def callback(data):
    global loc_acc
    global pose
    loc_acc = data.pose.covariance[0]+data.pose.covariance[7]
    pose = data

class localization():
    
    def __init__(self,wait=0.0):
        # amcl initialization 
        global loc_acc
        global pose
        loc_acc = 1.0
        rospy.init_node('amcl_local',anonymous=True)
        print('start')
        os.system("rosservice call /global_localization '{}'")
        rospy.Subscriber('amcl_pose',PoseWithCovarianceStamped,callback)
        rotate_pub = rospy.Publisher('base_velocity',Twist,queue_size=10)
        rotate = geometry_msgs.msg.Twist()
        rotate.angular.z = -1
        pose_pub = rospy.Publisher('laser_2d_correct_pose',PoseWithCovarianceStamped,queue_size=10)  # initialpose


        while loc_acc>0.01 and not rospy.is_shutdown():
            rotate_pub.publish(rotate)
            print(loc_acc)
            print('loop')
        

        print('out of loop,wait....')
        rospy.sleep(2.0)
        print('update the pose')
        rospy.Subscriber('amcl_pose',PoseWithCovarianceStamped,callback)
        pose_pub.publish(pose)
        rospy.sleep(5.0)
        print('update the pose second times')
        rospy.Subscriber('amcl_pose',PoseWithCovarianceStamped,callback)
        pose_pub.publish(pose)
        rospy.sleep(5.0)
        print('finish')
        
        os.system('rosnode kill /amcl_node')
        rospy.sleep(2.0)
        sys.exit()



if __name__ == '__main__':
    localization()
