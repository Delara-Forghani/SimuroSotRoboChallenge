#!/usr/bin/env python

import rospy
import roslib
import math
from sensor_msgs.msg import Image
from std_msgs.msg import *
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from system_messages.msg import Coordination
from geometry_msgs.msg import Twist
from vision import Vision
import time

class target_Coordination:

    def __init__(self):
        self.position=None
        self.twist = Twist()

    def subs_and_pubs(self):
         self.twist_pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist,queue_size=10)
         self.target_point_sub=rospy.Subscriber("/target/coordination",Coordination,self.coordinate_callback)

    def coordinate_callback(self,coordinate):
        self.position=coordinate
        #print("target point received",self.position.xCoordinate.data)
        self.move_towards_target()

    def move_towards_target(self):
        r = rospy.Rate(50) # Setting a rate (hz) at which to publish
        # while not rospy.is_shutdown():
        rospy.loginfo("Sending commands")
        print("center_x :",self.position.xCoordinate.data )
        if( 310 < self.position.xCoordinate.data < 330):
            print("Move towrads goal")
            self.twist.linear.x = 50
            self.twist.angular.z = 0
            self.twist_pub.publish(self.twist)
        elif(self.position.xCoordinate.data < 320):
            self.twist.linear.x = 0
            self.twist.angular.z = math.radians(5)
            self.twist_pub.publish(self.twist)
            print("less")
        elif(self.position.xCoordinate.data > 320):
            self.twist.linear.x = 0
            self.twist.angular.z = math.radians(-5)
            self.twist_pub.publish(self.twist)
            print("more")
        r.sleep() # Calling sleep to ensure the rate we set above



if __name__=='__main__':
    rospy.init_node('targetNode')
    robot_target=target_Coordination()
    robot_target.subs_and_pubs()
    rospy.spin()
