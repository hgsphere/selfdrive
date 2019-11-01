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
CAR_CENTER_RATIO = 95/256


class imageprocessor:

    def __init__(self):
        print('Init Transforms')
        self.compute_transforms()

        self.emStopD = EmergencyStopDetector()
        self.poller = None
        self.angle_last = 0

    def compute_transforms(self):
        # SETUP once
        self.hmg = getHomographyMatrix("color-lowres")
        self.invh = getHomographyMatrix("color-lowres", inverse=True)

    def displayImage(self, name, mat):
            cv.imshow(name, mat)
            return cv.waitKey(0)

    def get_half_target(self,target):
        (p0,p1) = target
        (bottomX, bottomY) = p0
        (avgTopX, avgTopY) = p1
        mid_X = (bottomX+avgTopX)/2
        # If the line is straight return half topY
        if bottomX-avgTopX == 0:
            return ((bottomX,bottomY),(mid_X,(480-avgTopY)/2)) # was 480 but should be 480, but retuning required

        # calc slope (should never be 0)
        m = float(avgTopY-bottomY)/ (avgTopX-bottomX)
        b = bottomY - m*bottomX

        # calc mid_Y
        mid_Y = m*mid_X + b
        #print(target)
        #print((bottomX,bottomY),(mid_X,mid_Y))
        return ((bottomX,bottomY),(mid_X,mid_Y))

    def calc_angle(self,target):
        # print(target)
        half_target = self.get_half_target(target)
        (p0,p1) = half_target
        (bottomX, bottomY) = p0
        (avgTopX, avgTopY) = p1 
        # (CAR_CENTER_RATIO*640,480)
        if bottomX-avgTopX == 0:
            #print('HELP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!111')
            return 0
        angle = np.arctan((CAR_CENTER_RATIO*640 - avgTopX)/(avgTopY-bottomY)) #was 640 but should be 424
        angle = 180*angle/np.pi
        # angle = 0
        return angle

    def getCorrectionAngle(self, frame):
        # frame is recent RGB image from car

        target = parseImage(frame, self.hmg, self.invh)

        if target is not None:
            angle = self.calc_angle(target)
            self.angle_last =  angle
            # print(angle)
        else:
            angle = self.angle_last
        return angle

    def getCrosswalk(self, frame):
        # while True:
        # frame = frame_stopDetectQ.get()
        CROSSWALK = False
        crossbox, crossLines = findStopLine(frame, self.hmg, self.invh)

        # Try thresholding the stop command if there is a cross walk close to the car
        # Might try returning avg y value of Crossbox and cross lines to know how close the car is
        # and make a state machine function to move till the y is close to the car
        if crossbox is not None:
            print(crossbox)
            avgY = 0
            for tar in crossbox:
                (X,Y) = tar[0]
                avgY = Y + avgY
            avgY = avgY/len(crossbox)
            print(avgY)
            if avgY > 225:
                CROSSWALK = True

        if crossLines is not None:
            print(crossLines)
            avgY = 0
            for tar in crossLines:
                (p1,p2) = tar
                #print(tar)
                #print(p1)
                (p1X,p1Y) = tuple(p1[0])
                (p2X,p2Y) = tuple(p2[0])
                avgY = avgY + p1Y + p2Y
            avgY = avgY/(len(crossLines)*2)
            print(avgY)
            if avgY > 225:
                CROSSWALK = True

        # If There are any crosswalks return true 
        #if (crossbox is not None) or (crossLines is not None):
        #    CROSSWALK = True

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
            stopDetect_routeManagerQ.put(stopLines)
            emStopDetect_routeManagerQ.put(False)
            count += 1


if __name__ == "__main__":
    print("Module is not runnable!")
