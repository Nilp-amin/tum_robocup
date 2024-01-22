#!/usr/bin/env python3


from __future__ import division

import rospy
import os
import sys
import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
import geometry_msgs.msg
import time




class Localization:
    def __init__(self):
        self.start=True



    def localization(self,req):
        
        os.system("roslaunch hsrb_amcl amcl_hsrb_realworld.launch")
        print('Service finished!')

        return EmptyResponse()




    def node_execute(self):

        print("Localization Node Execute\n")
        rospy.init_node('amcl_service_node',anonymous=True)
        s = rospy.Service('localization', Empty, self.localization)
       
        self.rate=rospy.Rate(2)
        

        while not rospy.is_shutdown():

            self.rate.sleep()



if __name__ == '__main__':
    node_instance = Localization()
    node_instance.node_execute()
