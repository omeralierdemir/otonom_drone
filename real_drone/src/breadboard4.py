#!/usr/bin/env python

import argparse
import imutils
import cv2
from imutils.video import VideoStream
from imutils.video import FPS
import time
import rospy
from std_msgs.msg import String 
# co

# load the video
camera = cv2.VideoCapture(1)
sayac = 0
sayac2 = 0
temp = 0

pub = rospy.Publisher('konum', String, queue_size=10)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(10)
fourcc = cv2.VideoWriter_fourcc(*'XVID')
video_writer = cv2.VideoWriter("output.avi", fourcc, 25, (640,480))
# keep looping
konum2 = "null"
while True:
    # grab the current frame and initialize the status text
    (grabbed, cap) = camera.read()
    frame = imutils.resize(cap, width=1100)
    ratio = cap.shape[1] / float(frame.shape[1])
    status = "No Targets"
    #print frame.shape
    # check to see if we have reached the end of the
    # video
    if not grabbed:
        break

    # convert the frame to grayscale, blur it, and detect edges
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    edged = cv2.Canny(blurred, 50, 150)

    # find contours in the edge map
    cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)


    for c in cnts:
        # approximate the contour
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.01 * peri, True)

        # ensure that the approximated contour is "roughly" rectangular
        if len(approx) >= 4 and len(approx) <= 6:
            # compute the bounding box of the approximated contour and
            # use the bounding box to compute the aspect ratio

            (x, y, w, h) = cv2.boundingRect(approx)
            aspectRatio = w / float(h)

            # compute the solidity of the original contour
            area = cv2.contourArea(c)
            hullArea = cv2.contourArea(cv2.convexHull(c))
            solidity = area / float(hullArea)

            # compute whether or not the width and height, solidity, and
            # aspect ratio of the con tour falls within appropriate bounds
            keepDims = w > 5 and h > 5
            keepSolidity = solidity > 0.9
            keepAspectRatio = aspectRatio >= 0.8 and aspectRatio <= 1.2

            # ensure that the contour passes all our tests
            if keepDims and keepSolidity and keepAspectRatio:
                edged2 = edged.copy()
                edged2[:, :] = 0
                edged2[y + 10:y + h - 10, x + 10:x + w - 10] = edged[y + 10:y + h - 10, x + 10:x + w - 10]
                cnts2 = cv2.findContours(edged2.copy(), cv2.RETR_EXTERNAL,
                                         cv2.CHAIN_APPROX_SIMPLE)
                cnts2 = imutils.grab_contours(cnts2)
                for j in cnts2:
                    peri2 = cv2.arcLength(j, True)
                    approx2 = cv2.approxPolyDP(j, 0.01 * peri2, True)
                    if len(approx2) >= 4 and len(approx2) <= 10:
                        x2, y2, w2, h2 = cv2.boundingRect(approx2)

                        if (x2 - x) > 0 and (y2 - y) > 0 and (h - h2) > 0 and (w - w2) > 0:
                            aspectRatio2 = w2 / float(h2)

                            # compute the solidity of the original contour
                            area2 = cv2.contourArea(j)
                            hullArea2 = cv2.contourArea(cv2.convexHull(j))
                            solidity2 = area2 / float(hullArea2)

                            # compute whether or not the width and height, solidity, and
                            # aspect ratio of the contour falls within appropriate bounds
                            keepDims2 = w2 > 5 and h2 > 5
                            keepSolidity2 = solidity2 > 0.9
                            keepAspectRatio2 = aspectRatio2 >= 0.8 and aspectRatio2 <= 1.2

                            if keepDims2 and keepSolidity2 and keepAspectRatio2:
                                cv2.drawContours(frame, [approx], -1, (0, 0, 255), 4)
                                cv2.drawContours(cap, [approx]*int(ratio), -1, (0, 0, 255), 4)
                                status = "Target(s) Acquired"
                                temp = 1

                                sayac = sayac + 1
                                print("4. kosul saglandi " , sayac)

                                M = cv2.moments(approx)
                                (cX, cY) = (int(M["m10"] // M["m00"]), int(M["m01"] // M["m00"]))
                                (aX, aY, radius) = (int(cX*ratio), int(cY*ratio), int(w*ratio))
                                konum = str(cX) + " " + str(cY) + " " + str(radius)
                                rospy.loginfo(str(radius))
                                pub.publish(konum)  
                                rate.sleep()
                                (startX, endX) = (int(cX - (w * 0.15)), int(cX + (w * 0.15)))
                                (startY, endY) = (int(cY - (h * 0.15)), int(cY + (h * 0.15)))
                                cv2.line(frame, (startX, aY), (endX, aY), (0, 0, 255), 3)
                                cv2.line(frame, (aX, startY), (aX, endY), (0, 0, 255), 3)

                                cv2.line(cap, (int(startX*ratio), int(cY*ratio)), (int(endX*ratio), int(cY*ratio)), (0, 0, 255), 3)
                                cv2.line(cap, (int(cX*ratio), int(startY*ratio)), (int(cX*ratio), int(endY*ratio)), (0, 0, 255), 3)
                                cv2.putText(frame, status, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
									(0, 0, 255), 2)

			
            

    if temp == 0:
		sayac2 = sayac2 + 1
		konum = "null" + " null" + " null"
		print(konum,sayac2)	
	
    temp = 0


    #video_writer.write(frame)
    #cv2.imshow("Frame", frame)
    #cv2.imshow("omer", cap)
    #key = cv2.waitKey(1) & 0xFF


    # if the 'q' key is pressed, stop the loop
    #if key == ord("q"):
     #   break

# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()

