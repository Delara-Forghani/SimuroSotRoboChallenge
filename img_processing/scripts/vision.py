#!/usr/bin/env python
import rospy
import roslib
from sensor_msgs.msg import Image
from std_msgs.msg import *
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from system_messages.msg import Coordination
from system_messages.msg import Rectangle
from system_messages.srv import PointsOfInterest
import time
from geometry import Point
from obstacleBorder import *


class Vision:

    def __init__(self):
        self.image=None
        self.bridge = CvBridge()
        self.camera_subscriber=None
        self.srv=None
        self.yellow_goal=True
        self.obstacle_positions=list()
        self.goal_position=Rectangle()
        self.obstacle_border=Rectangle()
        self.no_goal_found=False
    def set_pubs_subs(self):
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",Image,self.image_callback)
        self.srv=rospy.Service('points_of_interest',PointsOfInterest,self.service_callback)

    def service_callback(self,point):
        print(point.req.requsetNum)
        if(point.req.pick_yellow_goal == False):
            self.yellow_goal = False
        else:
            self.yellow_goal=True
        # del self.obstacle_positions[:]
        # self.setImageToCV()
        point.res.obstacle_positions=self.obstacle_positions
        point.res.goal_position=self.goal_position
        point.res.no_goal_found=self.no_goal_found


    def coordinate_callback(self,):
        print("target point received")


    def image_callback(self,image):
        #print("image received")
        self.image=image
        del self.obstacle_positions[:]
        self.setImageToCV()



    def setImageToCV(self):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="passthrough")
            #print("image converted to cv2")
            self.detectOpenCVImage(cv_image)
        except CvBridgeError as e:
            print(e)




    def detectOpenCVImage(self,cv_image):
        #print("detectOpenCVImage")
        im=cv_image
        crop_im=cv_image[360:480 , 165:475]
        frame = cv2.GaussianBlur(im, (3, 3), 0)
        crop_frame=cv2.GaussianBlur(crop_im, (3, 3), 0)


        # Switch image from BGR colorspace to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        crop_hsv=hsv[360:480 , 165:475]
        crop_gray=img_gray[360:480 , 165:475]




        # define range of purple color in HSV
        yellowMin = np.array([20, 100, 10])
        yellowMax =  np.array([30, 255, 255])

        blueMin = np.array([90, 100, 10])
        blueMax =  np.array([125, 255, 255])

        redMin= np.array([170, 100, 10])
        redMax= np.array([180, 255, 255])


        blue_mask = cv2.inRange(hsv, blueMin, blueMax)
        yellow_mask = cv2.inRange(hsv, yellowMin, yellowMax)
        red_mask = cv2.inRange(crop_hsv, redMin, redMax)


        # dilate makes the in range areas larger
        blue_mask = cv2.dilate(blue_mask, None, iterations=1)
        yellow_mask = cv2.dilate(yellow_mask, None, iterations=1)
        red_mask = cv2.dilate(red_mask, None, iterations=1)



        blue_result = cv2.bitwise_and(frame,frame, mask=blue_mask)
        yellow_result = cv2.bitwise_and(frame,frame, mask=yellow_mask)
        red_result = cv2.bitwise_and(crop_frame, crop_frame, mask=red_mask)


        blue_to_gray=cv2.cvtColor(blue_result,cv2.COLOR_BGR2GRAY)
        yellow_to_gray=cv2.cvtColor(yellow_result,cv2.COLOR_BGR2GRAY)
        red_to_gray = cv2.cvtColor(red_result, cv2.COLOR_BGR2GRAY)
        (blue_ret, blue_thresh) = cv2.threshold(blue_to_gray, 50 , 255, cv2.THRESH_BINARY)
        (yellow_ret, yellow_thresh) = cv2.threshold(yellow_to_gray, 70, 200, cv2.THRESH_BINARY)
        (red_ret, red_thresh) = cv2.threshold(red_to_gray, 10, 255, cv2.THRESH_BINARY)
        im1,red_contours, hierarchy1 = cv2.findContours(red_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        im2,yellow_contours, hierarchy2 = cv2.findContours(yellow_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        im3,blue_contours, hierarchy3 = cv2.findContours(blue_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


        if(len(red_contours)> 0):
            cnt = max(red_contours, key=cv2.contourArea)
            red_contours.sort(key=cv2.contourArea)

            #print("here is red_contours :" , len(red_contours))
            for i in red_contours:


                rect = cv2.boundingRect(i)
                x,y,w,h = rect
                cv2.rectangle(red_result,(x,y),(x+w,y+h),(255,0,0),2)


                # ****************
                self.obstacle_border.left_top_coord.x=x
                self.obstacle_border.left_top_coord.y=y

                self.obstacle_border.right_top_coord.x=x+w
                self.obstacle_border.right_top_coord.y=y

                self.obstacle_border.left_bot_coord.x=x
                self.obstacle_border.left_bot_coord.y=y+h

                self.obstacle_border.right_bot_coord.x=x+w
                self.obstacle_border.right_bot_coord.y=y+h

                self.obstacle_positions.append(self.obstacle_border)
        #print(len(self.obstacle_positions))
        if(self.yellow_goal == True):
            if(len(yellow_contours) == 0):
                self.no_goal_found=True
            elif(len(yellow_contours) > 0):
                yellow_cnt = max(yellow_contours, key=cv2.contourArea)
                if(cv2.contourArea(yellow_cnt) == 0):
                    self.no_goal_found=True

                else:
                    self.no_goal_found=False
                    rect = cv2.boundingRect(yellow_cnt)
                    x,y,w,h = rect
                    cv2.rectangle(yellow_result,(x,y),(x+w,y+h),(255,0,0),2)
                    self.goal_position.left_top_coord.x=x
                    self.goal_position.left_top_coord.y=y

                    self.goal_position.right_top_coord.x=x+w
                    self.goal_position.right_top_coord.y=y

                    self.goal_position.left_bot_coord.x=x
                    self.goal_position.left_bot_coord.y=y+h

                    self.goal_position.right_bot_coord.x=x+w
                    self.goal_position.right_bot_coord.y=y+h
        # print(self.no_goal_found)
        # print((self.goal_position.left_top_coord.x , self.goal_position.left_top_coord.y) , (self.goal_position.right_top_coord.x , self.goal_position.right_top_coord.y) ,
        # (self.goal_position.left_bot_coord.x , self.goal_position.left_bot_coord.y) , (self.goal_position.right_bot_coord.x , self.goal_position.right_bot_coord.y))

        if(self.yellow_goal == False):
            if(len(blue_contours) == 0):
                self.no_goal_found = True
            elif(len(blue_contours) > 0):
                blue_cnt = max(blue_contours, key=cv2.contourArea)
                if(cv2.contourArea(blue_cnt) == 0):
                    self.no_goal_found=True
                else:
                    self.no_goal_found=False
                    rect = cv2.boundingRect(blue_cnt)
                    x,y,w,h = rect
                    cv2.rectangle(blue_result,(x,y),(x+w,y+h),(255,255,0),2)
                    self.goal_position.left_top_coord.x=x
                    self.goal_position.left_top_coord.y=y

                    self.goal_position.right_top_coord.x=x+w
                    self.goal_position.right_top_coord.y=y

                    self.goal_position.left_bot_coord.x=x
                    self.goal_position.left_bot_coord.y=y+h

                    self.goal_position.right_bot_coord.x=x+w
                    self.goal_position.right_bot_coord.y=y+h




        cv2.imshow('frame',frame)
        cv2.imshow('crop_frame',crop_frame)
        cv2.imshow('red_result', red_result)
        cv2.imshow('blue_result', blue_result)
        cv2.imshow('yellow_result', yellow_result)


        k = cv2.waitKey(1)


if __name__=='__main__':
    rospy.init_node('visionNode')
    robot_vision=Vision()
    robot_vision.set_pubs_subs()
    rospy.spin()
