#!/usr/bin/env python
import rospy
import roslib
from std_msgs.msg import *
from geometry_msgs.msg import *


class Move:
    def __init__(self):
        self.image = None
        self.sub = rospy.Subscriber("state",std_msgs.msg.Int8, self.getmove)
        self.pub = rospy.Publisher("/cmd_vel_mux/input/navi",geometry_msgs.msg.Twist, queue_size=10)

    def getmove(self, i):
        tcmd = geometry_msgs.msg.Twist()
        print ("State: ", i.data)
        if i.data == 0:
            tcmd.linear.x = 0.3
        elif i.data == 1:
            tcmd.angular.z = 0.3
        elif i.data == 2:
            tcmd.angular.z = -0.5
        elif i.data == 3:
            tcmd.linear.x = 0.5
            tcmd.angular.z = 0.2
        elif i.data == 4:
            tcmd.linear.x = 0.5
            tcmd.angular.z = -0.2
        print (tcmd.linear.x, tcmd.angular.z)
        self.pub.publish(tcmd)


if __name__ == '__main__':
    rospy.init_node('move')
    robot_move = Move()
    rospy.spin()
