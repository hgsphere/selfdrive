#!/usr/bin/bash

import os
import sys
import time
import cv2 as cv
import numpy as np
import pyrealsense2 as rs

sys.path.append(os.path.abspath(os.getcwd()))
sys.path.append(os.path.abspath("../camera/"))
sys.path.append(os.path.abspath("../systemStructure/"))
from calibrate import getHomographyMatrix
from findLines import parseImage, displayImage, showHeading, cleanupImage, getContours
from findStopLine import findStopLine, drawCrossBox, drawCrossLines
from pollers import Pollers
from EmergencyStopDetector import EmergencyStopDetector

# RBG_IMAGE = 'frame.jpeg'
VIEW = False
CAR_CENTER_RATIO = 24/64


class imageprocessor:

    def __init__(self):
        print('Init Transforms')
        self.compute_transforms()

        self.emStopD = EmergencyStopDetector()
        self.poller = None

    def compute_transforms(self):
        # SETUP once
        self.hmg = getHomographyMatrix("color-lowres")
        self.invh = getHomographyMatrix("color-lowres", inverse=True)

    def displayImage(self, name, mat):
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

    def getCorrectionAngle(self, frame):
        # frame is recent RGB image from car

        target = parseImage(frame, self.hmg, self.invh)

        if target is not None:
            angle = self.calc_angle(target)
            # print(angle)
        else:
            angle = -666
        return angle

    def getCrosswalk(self, frame):
        # while True:
        # frame = frame_stopDetectQ.get()
        CROSSWALK = False
        crossbox, crossLines = findStopLine(frame, self.hmg, self.invh)

        if (crossbox is not None) or (crossLines is not None):
            CROSSWALK = True
        return CROSSWALK

    def runImageProcessing(self, laneDetect_routeManagerQ, stopDetect_routeManagerQ, emStopDetect_routeManagerQ):
        self.poller = Pollers()
        # print("starting delay for image processing")
        # for i in range(0,5):
        #     color, depth = self.poller.pollFrame()
        print("starting actual image processing")
        count = 0
        while True:
            color, depth = self.poller.pollFrame()
            # print("count: {}".format(count))
            if (color is None) and (depth is None):
                continue
            # cv.imwrite("testFrame{}.jpeg".format(count), color)

            angle = self.getCorrectionAngle(color)
            # emStop = self.emStopD.detectStop(depth)
            stopLines = self.getCrosswalk(color)
            laneDetect_routeManagerQ.put(angle)
            stopDetect_routeManagerQ.put(False)
            emStopDetect_routeManagerQ.put(False)
            count += 1


if __name__ == "__main__":
    print("Module is not runnable!")
