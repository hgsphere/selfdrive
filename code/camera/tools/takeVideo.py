#!/usr/bin/python3
#record_video.py
# USAGE: python3 record_video.py --output output/my_output.avi

# import the necessary packages
import argparse
import imutils
import cv2
 
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-o", "--output", required=True,
	help="path to output video")
ap.add_argument("-c", "--confidence", type=float, default=0.5,
	help="minimum probability to filter weak detections")
args = vars(ap.parse_args())	

vs = cv2.VideoCapture("/dev/video2", cv2.CAP_V4L) # ls -ltr /dev/video*
writer = None
(W, H) = (None, None)

while True:
    # read the next frame from the file
    (grabbed, frame) = vs.read()
 
    # if the frame was not grabbed, then we have reached the end
    # of the stream
    if not grabbed:
        break

    if W is None or H is None:
        (H, W) = frame.shape[:2]

    # check if the video writer is None
    if writer is None:
        # initialize our video writer
        fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        writer = cv2.VideoWriter(args["output"], fourcc, 30,
            (frame.shape[1], frame.shape[0]), True)
 
    # write the output frame to disk
    writer.write(frame)

writer.release()
vs.release()
