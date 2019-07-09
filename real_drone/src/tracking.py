#!/usr/bin/env python



import argparse
import imutils
import cv2
from imutils.video import VideoStream
from imutils.video import FPS
import time

# co

# load the video
check = 0
camera = cv2.VideoCapture(0)
fps = None
initBB = None
OPENCV_OBJECT_TRACKERS = {
    "csrt": cv2.TrackerCSRT_create,
    "kcf": cv2.TrackerKCF_create,
    "boosting": cv2.TrackerBoosting_create,
    "mil": cv2.TrackerMIL_create,
    "tld": cv2.TrackerTLD_create,
    "medianflow": cv2.TrackerMedianFlow_create,
    "mosse": cv2.TrackerMOSSE_create
}

tracker = OPENCV_OBJECT_TRACKERS["csrt"]()
count = 0

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi',fourcc, 20.0, (500,375))
# keep looping
while True:
    # grab the current frame and initialize the status text
    (grabbed, frame) = camera.read()
    status = "No Targets"

    # check to see if we have reached the end of the
    # video
    if not grabbed:
        break
    frame = imutils.resize(frame, width=1000)
    (H, W) = frame.shape[:2]
    print(H,W)

    # convert the frame to grayscale, blur it, and detect edges
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    edged = cv2.Canny(blurred, 50, 150)

    # find contours in the edge map
    cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    if initBB is not None:
        # grab the new bounding box coordinates of the object
        (success, box) = tracker.update(frame)
        count = 1 + count
        # check to see if the tracking was a success

        (x, y, w, h) = [int(v) for v in box]
        cv2.rectangle(frame, (x, y), (x + w, y + h),
                      (0, 255, 0), 2)

        if count == 30:
            initBB = None
            print("fps ",success,box, initBB)
            count = 0
        # update the FPS counter
        fps.update()
        fps.stop()

        # initialize the set of information we'll be displaying on
        # the frame
        info = [
            ("Tracker", "kcf"),
            ("Success", "Yes" if success else "No"),
            ("FPS", "{:.2f}".format(fps.fps())),
        ]

        # loop over the info tuples and draw them on our frame
        for (i, (k, v)) in enumerate(info):
            text = "{}: {}".format(k, v)
            cv2.putText(frame, text, (10, H - ((i * 20) + 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)


    if initBB is None:
        # loop over the contours
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
                keepSolidity = solidity > 0.8
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
                                keepSolidity2 = solidity2 > 0.8# oranlarla oynandi hassiyeti  icin
                                keepAspectRatio2 = aspectRatio2 >= 0.8 and aspectRatio2 <= 1.2

                                if keepDims2 and keepSolidity2 and keepAspectRatio2:
                                    cv2.drawContours(frame, [approx], -1, (0, 0, 255), 4)
                                    status = "Target(s) Acquired"

                                    initBB = (x-10,y-10,w+10,h+10)
                                    print(initBB)

                                    # start OpenCV object tracker using the supplied bounding box
                                    # coordinates, then start the FPS throughput estimator as well
                                    tracker = OPENCV_OBJECT_TRACKERS["csrt"]()
                                    tracker.init(frame, initBB)
                                    if check == 0:
                                        fps = FPS().start()
                                    else:
                                        fps.update()

                                    # compute the center of the contour region and draw the
                                    # crosshairs
                                    M = cv2.moments(approx)
                                    (cX, cY) = (int(M["m10"] // M["m00"]), int(M["m01"] // M["m00"]))
                                    (startX, endX) = (int(cX - (w * 0.15)), int(cX + (w * 0.15)))
                                    (startY, endY) = (int(cY - (h * 0.15)), int(cY + (h * 0.15)))
                                    cv2.line(frame, (startX, cY), (endX, cY), (0, 0, 255), 3)
                                    cv2.line(frame, (cX, startY), (cX, endY), (0, 0, 255), 3)


        # draw the status text on the frame
                                    cv2.putText(frame, status, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                                                                                    (0, 0, 255), 2)

    # show the frame and record if a key is pressed
    out.write(frame)
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break

# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()