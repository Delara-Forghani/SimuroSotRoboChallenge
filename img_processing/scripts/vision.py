#!/usr/bin/env python
import rospy
import roslib
from sensor_msgs.msg import Image
from std_msgs.msg import *
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from system_messages.msg import Coordination
import time
from geometry import Point
from obstacleBorder import *


class Vision:
    def __init__(self):
        self.image=None
        self.obstacle_boundaries=list()
        self.bridge = CvBridge()
        self.left_top_point=Coordination()
        self.right_top_point=Coordination()
        self.left_bot_point=Coordination()
        self.right_bot_point=Coordination()
        self.left_flag = False
        self.right_flag = False
        self.state = 0
        self.cmd = 0
    def set_pubs_subs(self):
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",Image,self.image_callback)
        self.goal_point_pub = rospy.Publisher("/target/coordination",Coordination,queue_size=1)
        self.target_point_sub=rospy.Subscriber("/target/coordination",Coordination,self.coordinate_callback)
        self.pub_state = rospy.Publisher('/state', std_msgs.msg.Int8, queue_size=10)


    def coordinate_callback(self,):
        print("target point received")

    def image_callback(self,image):
        #print("image received")
        # for i in range len(self.obstacle_boundaries):
        #     self.obstacle_boundaries=list()

        del self.obstacle_boundaries[:]
        #print(len(self.obstacle_boundaries))
        self.image=image
        self.setImageToCV()
        self.pub_state.publish(self.cmd)


    def setImageToCV(self):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="passthrough")
            #print("image converted to cv2")
            self.detectOpenCVImage(cv_image)
        except CvBridgeError as e:
            print(e)

    #def distance(self , )


    def detectOpenCVImage(self,cv_image):
        #print("detectOpenCVImage")
        im =cv_image[360:480 , 165:475]
        frame = cv2.GaussianBlur(im, (3, 3), 0)


        # Switch image from BGR colorspace to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)



        # define range of purple color in HSV
        yellowMin = np.array([20, 100, 10])
        yellowMax =  np.array([30, 255, 255])

        #blueMin = np.array([90, 100, 10])        it may be interchanged with yellow
        #blueMax =  np.array([125, 255, 255])     it may be interchanged with yellow


        redMin= np.array([170, 100, 10])
        redMax= np.array([180, 255, 255])
        # Sets pixels to white if in purple range, else will be set to black



        greenMin=np.array([40,100,10])
        greenMax=np.array([90,255,255])


        grayMin=np.array([18,4,39])
        grayMax=np.array([28,10,100])





        #blue_mask = cv2.inRange(hsv, blueMin, blueMax)      it may be interchanged with yellow
        yellow_mask = cv2.inRange(hsv, yellowMin, yellowMax)
        red_mask = cv2.inRange(hsv, redMin, redMax)
        green_mask=cv2.inRange(hsv,greenMin , greenMax)
        gray_mask =cv2.inRange(hsv,grayMin,grayMax)




        yellow_result = cv2.bitwise_and(frame, frame, mask=yellow_mask)
        red_result = cv2.bitwise_and(frame, frame, mask=red_mask)
        green_result=cv2.bitwise_and(frame , frame ,mask=green_mask)
        gray_result=cv2.bitwise_and(frame , frame , mask=gray_mask)
        red_green_mask=cv2.bitwise_or(red_mask , green_mask)
        red_green_result=cv2.bitwise_and(frame ,frame ,mask=red_green_mask)


        # dilate makes the in range areas larger
        yellow_mask = cv2.dilate(yellow_mask, None, iterations=1)
        red_mask = cv2.dilate(red_mask, None, iterations=1)
        red_green_mask=cv2.dilate(red_green_mask , None ,iterations=1)


        edges = cv2.Canny(red_mask, 100, 200)
        yellow_to_gray=cv2.cvtColor(yellow_result,cv2.COLOR_BGR2GRAY)
        red_to_gray = cv2.cvtColor(red_result, cv2.COLOR_BGR2GRAY)
        redgreen_to_gray = cv2.cvtColor(red_green_result,cv2.COLOR_BGR2GRAY)
        (yellow_ret, yellow_thresh) = cv2.threshold(yellow_to_gray, 50, 200, cv2.THRESH_BINARY)
        (red_ret, red_thresh) = cv2.threshold(red_to_gray, 10, 255, cv2.THRESH_BINARY)
        (rg_ret ,rg_thresh) = cv2.threshold(redgreen_to_gray,50,255,cv2.THRESH_BINARY)
        im1,red_contours, hierarchy1 = cv2.findContours(red_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        im2,yellow_contours, hierarchy2 = cv2.findContours(yellow_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        im3,rg_contours, hierarchy3 = cv2.findContours(rg_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)





        if(len(rg_contours) > 0):
            cnt=max(rg_contours , key=cv2.contourArea)
            cv2.drawContours(red_green_result,[cnt], -1, (0, 255, 0), 3)
        if(len(red_contours)> 0):
            cnt = max(red_contours, key=cv2.contourArea)
            red_contours.sort(key=cv2.contourArea)

            #print("here is red_contours :" , len(red_contours))
            for i in red_contours:
                hull = cv2.convexHull(i)
                cv2.drawContours(red_result, [hull], -1, (0, 0, 255), 1)
                epsilon = 0.01 * cv2.arcLength(i, True)
                approx = cv2.approxPolyDP(i, epsilon, True)
                cv2.drawContours(red_result, [approx], -1, (255,0, 0), 1)




                rect = cv2.boundingRect(i)
                x,y,w,h = rect
                cv2.rectangle(red_result,(x,y),(x+w,y+h),(255,0,0),2)




                left_top=Point(x,y)
                right_top=Point(x+w,y)
                left_bot=Point(x,y+h)
                right_bot=Point(x+w,y+h)
                border=Border(left_top,right_top,left_bot,right_bot)
                self.obstacle_boundaries.append(border)
                border.print_points()


                # ****************
                self.left_top_point.xCoordinate.data=left_top.x
                self.left_top_point.yCoordinate.data=left_top.y

                self.right_top_point.xCoordinate.data=right_top.x
                self.right_top_point.yCoordinate.data=right_top.y

                self.left_bot_point.xCoordinate.data=left_bot.x
                self.left_bot_point.yCoordinate.data=left_bot.y

                self.right_bot_point.xCoordinate.data=right_bot.x
                self.right_bot_point.yCoordinate.data=right_bot.y
                print("*******")
                print((self.left_top_point.xCoordinate.data , self.left_top_point.yCoordinate.data))

            #print(len(self.obstacle_boundaries))
            for i in range (0 , len(self.obstacle_boundaries)):
                if(self.obstacle_boundaries[i].get_right_bot().x == 310 ):
                    self.right_flag = True
                if(self.obstacle_boundaries[i].get_left_bot().x == 0 ):
                    self.left_flag= True


            if(self.right_flag == True and self.left_flag == True):
                self.cmd = 0
            elif(self.right_flag == True and self.left_flag == False):
                self.cmd = 1
            elif(self.right_flag == False and self.left_flag == True):
                self.cmd = 2
            elif(self.right_flag == False and self.left_flag == False):
                self.cmd = 3
            print(self.cmd)

        cv2.drawContours(yellow_result,yellow_contours, -1, (0, 255, 0), 3)







        cv2.imshow('frame',frame)
        cv2.imshow('red_result', red_result)

        k = cv2.waitKey(1)


if __name__=='__main__':
    rospy.init_node('visionNode')
    robot_vision=Vision()
    robot_vision.set_pubs_subs()
    rospy.spin()
