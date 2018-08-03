# Standard imports
import cv2
import numpy as np

class ImageProcess:
    # Read image
    im = cv2.imread("download.jpeg", cv2.IMREAD_COLOR)
    #type=im.type()
    #print(im.dtype)
    #print(im.shape)
    frame = cv2.GaussianBlur(im, (3, 3), 0)

    # Switch image from BGR colorspace to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of purple color in HSV
    yellowMin = (20, 100, 100)
    yellowMax = (30, 255, 255)

    blueMin = (90, 100, 100)
    blueMax = (125, 255, 255)

    redMin=(0, 100, 100)
    redMax=(10, 255, 255)

    red2Min=(160 , 100 ,100)
    red2Max=(179 , 255, 255)

    greenMax=()
    greenMin=()
    # Sets pixels to white if in purple range, else will be set to black

    mask1 = cv2.inRange(hsv, blueMin, blueMax)
    mask2 = cv2.inRange(hsv, yellowMin, yellowMax)
    mask3 = cv2.inRange(hsv, redMin, redMax)
    mask4 = cv2.inRange(hsv, red2Min ,red2Max)


    # Bitwise-AND of mask and yellow only image - only used for display
    # res2 = cv2.bitwise_and(frame, frame, mask=mask2)


    mask = cv2.bitwise_or(mask1, mask2)
    maskFinal =cv2.bitwise_or(mask, mask3)
    maskFinal2 = cv2.bitwise_or(maskFinal, mask4)
    res = cv2.bitwise_and(frame, frame, mask=maskFinal2)

    #    mask = cv2.erode(mask, None, iterations=1)
    # commented out erode call, detection more accurate without it

    # dilate makes the in range areas larger
    maskFinal2 = cv2.dilate(maskFinal2, None, iterations=1)

    # Set up the SimpleBlobdetector with default parameters.
    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 256;

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 300
    params.maxArea = 1000

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.2

    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.5

    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.5

    detector = cv2.SimpleBlobDetector_create(params)

    # Detect blobs.
    keypoints = detector.detect(maskFinal)
    # if keypoints:
    #     print
    #     "found %d blobs" % len(keypoints)
    #     if len(keypoints) > 4:
    #         # if more than four blobs, keep the four largest
    #         keypoints.sort(key=(lambda s: s.size))
    #         keypoints = keypoints[0:3]
    #     else:
    #         print
    #         "no blobs"

            # Draw green circles around detected blobs
    im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (255, 0, 0),
                                        cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # edges = cv2.Canny(res, 100, 200)
    # imgray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    # (ret, thresh) = cv2.threshold(imgray, 127, 255, 0)
    # im2,contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # cnt=contours[0]
    # M=cv2.moments(cnt)
    # print(M)
    # cv2.drawContours(res, contours, -1, (0, 255, 0), 3)

    # open windows with original image, mask, res, and image with keypoints marked
    cv2.imshow('frame', frame)
    # cv2.imshow('mask', mask)
    # cv2.imshow('mask2', mask2)
    cv2.imshow('res', res)
    # cv2.imshow('im2',im2)
    #cv2.imshow('edges', edges)

    # cv2.imshow('res2', res2)
    cv2.imshow("Keypoints" , im_with_keypoints)

    k = cv2.waitKey(0)
