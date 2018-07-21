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



class Vision:
    def __init__(self):
        self.image=None
        self.bridge = CvBridge()
        self.goalCoordination=Coordination()

    def set_pubs_subs(self):
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",Image,self.image_callback)
        self.goal_point_pub = rospy.Publisher("/target/coordination",Coordination,queue_size=1)
        self.target_point_sub=rospy.Subscriber("/target/coordination",Coordination,self.coordinate_callback)

    def coordinate_callback(self,):
        print("target point received")

    def image_callback(self,image):
        #print("image received")
        self.image=image
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
        im =cv_image
        frame = cv2.GaussianBlur(im, (3, 3), 0)


        # Switch image from BGR colorspace to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)



        # define range of purple color in HSV
        yellowMin = np.array([20, 100, 10])
        yellowMax =  np.array([30, 255, 255])

        blueMin = np.array([90, 100, 10])
        blueMax =  np.array([125, 255, 255])

        redMin= np.array([170, 100, 10])
        redMax= np.array([180, 255, 255])
        # Sets pixels to white if in purple range, else will be set to black

        blue_mask = cv2.inRange(hsv, blueMin, blueMax)
        yellow_mask = cv2.inRange(hsv, yellowMin, yellowMax)
        red_mask = cv2.inRange(hsv, redMin, redMax)



        blu_yell_mask = cv2.bitwise_or(blue_mask,yellow_mask)
        mask_final =cv2.bitwise_or(blu_yell_mask,red_mask)
        result = cv2.bitwise_and(frame, frame, mask=mask_final)


        # dilate makes the in range areas larger
        mask_final = cv2.dilate(mask_final, None, iterations=1)

        # Set up the SimpleBlobdetector with default parameters.
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold =10;
        params.maxThreshold = 30;

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 100


        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.01

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.87

        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.01

        detector = cv2.SimpleBlobDetector_create(params)

        # Detect blobs.
        keypoints = detector.detect(img_gray)
        if keypoints:
            print
            "found %d blobs" % len(keypoints)
            if len(keypoints) > 4:
                # if more than four blobs, keep the four largest
                keypoints.sort(key=(lambda s: s.size))
                keypoints = keypoints[0:3]
            else:
                print
                "no blobs"

                # Draw green circles around detected blobs
        im_with_keypoints = cv2.drawKeypoints(img_gray, keypoints, np.array([]), (0, 255, 0),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        edges = cv2.Canny(result, 100, 200)
        imgray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        (ret, thresh) = cv2.threshold(imgray, 10, 255, cv2.THRESH_BINARY)
        im2,contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        print("contours",len(contours))
        # if len(contours) == 0:
        #     r=rospy.Rate(50)
        #     self.goalCoordination.xCoordinate.data = 0
        #     self.goalCoordination.yCoordinate.data = 224
        #     center = (0,224)
        #     print "Here is the center: %s %s" % (self.goalCoordination.xCoordinate,self.goalCoordination.yCoordinate)
        #     print (result.shape)
        #     self.goal_point_pub.publish(self.goalCoordination)
        #     r.sleep()
        # if len(contours) > 0 :
        #     c = max(contours , key=cv2.contourArea)
    	#     ((x,y),radius) = cv2.minEnclosingCircle(c)
    	#     M = cv2.moments(c)
        #     if( M["m00"] == 0):
        #         print("Float division by zero :) ")
        #     else:
        #         r=rospy.Rate(50)
        #         self.goalCoordination.xCoordinate.data = (M["m10"] / M["m00"])
        #         self.goalCoordination.yCoordinate.data = (M["m01"] / M["m00"])
    	#         center = ((M["m10"] / M["m00"]), (M["m01"] / M["m00"]))
    	#         print "Here is the center: %s %s" % (self.goalCoordination.xCoordinate,self.goalCoordination.yCoordinate)
        #         print (result.shape)
        #         self.goal_point_pub.publish(self.goalCoordination)
        #         r.sleep()
        #         #print("X and Y published")


        for c in contours:
            rect = cv2.boundingRect(c)
            #if rect[2] < 100 or rect[3] < 100: continue
            x,y,w,h = rect
            cv2.rectangle(result,(x,y),(x+w,y+h),(255,0,0),2)


        cv2.drawContours(result, contours, -1, (0, 255, 0), 3)
        cv2.circle(result, (int(self.goalCoordination.xCoordinate.data),int(self.goalCoordination.yCoordinate.data)), 7, (255, 255, 255), -1)


        #print(np.where((result == red_mask)))
        # max=0
        # min=640
        # if (cv2.countNonZero(red_mask)!= 0):
        #     for i in range(0, red_mask.shape[0]):
        #             for j in range(0, red_mask.shape[1]):
        #                 if(red_mask[i, j] == 255):
        #                     if(j > max):
        #                         max=j
        #                     if( j < min):
        #                         min=j
        # print(min)
        # print(max)
        # nonzero = cv2.countNonZero(red_mask)
        # print(nonzero)
        # #white_coords = np.argwhere(red_mask == 255)
        # white_coords=cv2.findNonZero(red_mask)
        # print(white_coords)


        cv2.imshow('frame',frame)
        cv2.imshow('thresh',thresh)
        cv2.imshow('res', result)
        cv2.imshow('edges', edges)
        cv2.imshow('red_mask' , red_mask)
        cv2.imshow('im_with_keypoints',im_with_keypoints)
        k = cv2.waitKey(1)


if __name__=='__main__':
    rospy.init_node('visionNode')
    robot_vision=Vision()
    robot_vision.set_pubs_subs()
    rospy.spin()
