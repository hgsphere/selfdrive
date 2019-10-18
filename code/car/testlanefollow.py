#!/usr/bin/bash

import os
import sys
import time
import json
import cv2 as cv
import numpy as np
import pyrealsense2 as rs

sys.path.append(os.path.abspath(os.getcwd()))
# sys.path.append(os.path.abspath("../utils/"))
sys.path.append(os.path.abspath("../camera/"))
from calibrate import getHomographyMatrix
from findLines import parseImage, displayImage, showHeading, cleanupImage, getContours
from findStopLine import findStopLine, drawCrossBox, drawCrossLines

# RBG_IMAGE = 'frame.jpeg'
VIEW = False
CAR_CENTER_RATIO = 9/16

class lanefollower:

    def __init__(self):
        print('Init Transforms')
        self.compute_transforms()

        print('Init Camera')
        self.config_frames_pipeline()

    def compute_transforms(self):
        # SETUP once
        self.hmg = getHomographyMatrix("color")
        self.invh = getHomographyMatrix("color", inverse=True)

    def config_frames_pipeline(self):
        config = rs.config()
        shape = (640, 480)
        frame_rate = 30
        resolution = (640, 480)
        outPath = 'test.avi'
        self.out = cv.VideoWriter(outPath, cv.VideoWriter_fourcc('M','J','P','G'), frame_rate, resolution)

        config.enable_stream(rs.stream.depth, shape[0], shape[1], rs.format.z16, frame_rate)
        config.enable_stream(rs.stream.color, shape[0], shape[1], rs.format.bgr8, frame_rate)
        self.pipeline = rs.pipeline()

        self.pipeline.start(config)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()
        # if not depth: 
        #     continue

        colorData = np.asanyarray(color.get_data())
        depthData = np.asanyarray(depth.get_data())

        # Render images
        depth_colormap = np.asanyarray(cv.applyColorMap(
                cv.convertScaleAbs(depthData, alpha=0.03), cv.COLORMAP_JET))
        
        return colorData, depth_colormap

    def displayImage(self,name, mat):
            cv.imshow(name, mat)
            return cv.waitKey(0)

    def calc_angle(self,target):
        # print(target)
        (p0,p1) = target
        (bottomX, bottomY) = p0
        (avgTopX, avgTopY) = p1 
        # (CAR_CENTER_RATIO*640,480)
        if bottomX-avgTopX == 0:
            return 0
        angle = np.arctan((CAR_CENTER_RATIO*640 - avgTopX)/(avgTopY-bottomY))
        angle = 180*angle/np.pi
        # angle = 0
        return angle

    def proc(self,frame):
        # frame is recent RGB image from car
        # frame = cv.imread(RBG_IMAGE, cv.IMREAD_COLOR)
        # target is (leftPts, rightPts)

        CROSSWALK = False

        target = parseImage(frame, self.hmg, self.invh)

        overlay = showHeading(target, frame)

        #displayImage('overlay',overlay)

        if target is not None:
            angle = self.calc_angle(target)
        else:
            angle = 0
        # if debug:
        #     target, overlay = target
        # else:
        #     pass
        #     overlay = showHeading(target, frame)
        
        crossbox, crossLines = findStopLine(frame, self.hmg, self.invh)



        if (crossbox is not None) or (crossLines is not None):
            CROSSWALK = True


        if VIEW:
            overlay = showHeading(target, frame)
            self.displayImage('overlay',overlay)
            if CROSSWALK:
                overlay = drawCrossBox(overlay, crossbox)
                overlay = drawCrossLines(overlay, crossLines)
            displayImage('overlay2',overlay)

        return angle,CROSSWALK

def testcapture():
    print("Test Camera Capture and detection")
    lf = lanefollower()
    time.sleep(1) # wait for warm up

    while True:
        try:

            rbg,depth = lf.get_frame()
            angle,CROSSWALK = lf.proc(rbg)
            
            # print(angle,CROSSWALK)
            result = {
                "angle" : angle,
                "crosswalk": CROSSWALK,
            }
            print(json.dumps(result, indent=2), ",")
        except Exception as e:
            raise e
            break
    


if __name__ == "__main__":
    testcapture()
