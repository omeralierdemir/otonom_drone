#!/usr/bin/env python

from collections import deque
from imutils.video import VideoStream
import numpy as np

import cv2

import time

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
from yolo import YOLO
from PIL import Image
import cv2
import numpy as np


time.sleep(2.0)

bridge = CvBridge()

vs = None

params = {
    "model_path": 'model_data/yolo-angle.h5',
    "anchors_path": 'model_data/drone_anchors.txt',
    "classes_path": 'model_data/drone_classes.txt',
    "score" : 0.5,
    "iou" : 0.4,
    "model_image_size" : (416, 416),
    "gpu_num" : 1,
}

def image_callback(ros_image):
    print 'got an image'
    global bridge
    global vs
  #convert ros_image into an opencv-compatible image
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    
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
    	
    
	
    cap = cv2.VideoCapture(0)
    yolo = YOLO(params)

    while(True):
        ret, frame = cap.read()

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(frame)

        r_image,data = yolo.detect_image(image)
		  print(data)	

        open_cv_image = np.array(r_image)
        open_cv_image = open_cv_image[:, :, ::-1].copy()

        cv2.imshow('frame', open_cv_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


    while not rospy.is_shutdown():
       
        print "while"
        frame = vs

        # handle the frame from VideoCapture or VideoStream
        
        # if we are viewing a video and we did not grab a frame,
        # then we have reached the end of the video
        if frame is None:
            break

       
       
            pub.publish(konum)  
            rate.sleep()                   
            time.sleep(0.1)

      
        # show the frame to our screen
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF

        # if the 'q' key is pressed, stop the loop
        if key == ord("q"):
            break

    
    # close all windows
    cv2.destroyAllWindows()
