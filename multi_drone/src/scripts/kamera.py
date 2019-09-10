#!/usr/bin/env python

from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

# construct the argument parse and parse the arguments

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)

# if a video path was not supplied, grab the reference
# to the webcam

#vs = cv2.VideoCapture(args["video"])

# allow the camera or video file to warm up
time.sleep(2.0)

bridge = CvBridge()

vs = None

pts = deque(maxlen=64)
log_kamera = open("log_kamera.txt", "w")
def image_callback(ros_image):
    
    global bridge
    global vs
  #convert ros_image into an opencv-compatible image
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        #print "sekilsin ha ", cv_image.shape
    
    except CvBridgeError as e:
        print(e)
    vs = cv_image
    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(3)


if __name__ == '__main__':
    rospy.init_node('image_converter', anonymous=True)
    image_sub = rospy.Subscriber("/iris_1/camera_red_iris/image_raw",Image, image_callback)
    pub = rospy.Publisher('konum', String, queue_size=10)
    
    rate = rospy.Rate(20)

    time.sleep(4.0)
    

   
    while not rospy.is_shutdown():
        # grab the current frame
        
        frame = vs

        # handle the frame from VideoCapture or VideoStream
        
        # if we are viewing a video and we did not grab a frame,
        # then we have reached the end of the video
        if frame is None:
            break

        # resize the frame, blur it, and convert it to the HSV
        # color space
        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        #print frame.shape , " bu kadar"
        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            konum = str(int(x)) + " " + str(int(y)) + " " + str(int(radius))
            log_kamera.write(konum+"\n")
            print(int(x),int(y),int(radius))
            pub.publish(konum)  
            rate.sleep()                   
            time.sleep(0.1)

            # only proceed if the radius meets a minimum size
            if radius > 5:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                    (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

        else:

            
            konum = "null null null"
            print konum
            pub.publish(konum)  

        # update the points queue
        pts.appendleft(center)

        # loop over the set of tracked points
        for i in range(1, len(pts)):
            # if either of the tracked points are None, ignore
            # them
            if pts[i - 1] is None or pts[i] is None:
                continue

            # otherwise, compute the thickness of the line and
            # draw the connecting lines
            thickness = int(np.sqrt( 64 / float(i + 1)) * 2.5)
            cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

        # show the frame to our screen
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF

        # if the 'q' key is pressed, stop the loop
        if key == ord("q"):
            break

    
    # close all windows
    log_kamera.close()
    cv2.destroyAllWindows()
