#!/usr/bin/bash

from os import (path as os_path,
                getcwd as os_getcwd)
from sys import (path as sys_path)
from cv2 import (imwrite as cv_imwrite,
                imshow as cv_imshow,
                waitKey as cv_waitKey)
from numpy import ( pi as np_pi,
                    arctan as np_arctan)

sys_path.append(os_path.abspath(os_getcwd()))
sys_path.append(os_path.abspath("../camera/"))
sys_path.append(os_path.abspath("../systemStructure/"))
from calibrate import getHomographyMatrix
from findLines import parseImage, displayImage, showHeading, cleanupImage, getContours, init_video
from findStopLine import findStopLine, drawCrossBox, drawCrossLines
from pollers import Pollers
from EmergencyStopDetector import EmergencyStopDetector

# RBG_IMAGE = 'frame.jpeg'
VIEW = False
#CAR_CENTER_RATIO = 140/256 #was 135
CAR_CENTER_RATIO = 95/256 # for incorrect angle 


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
            cv_imshow(name, mat)
            return cv_waitKey(0)

    def get_half_target(self,target):
        (p0,p1) = target
        (bottomX, bottomY) = p0
        (avgTopX, avgTopY) = p1
        mid_X = (bottomX+avgTopX)/2
        # If the line is straight return half topY
        if bottomX-avgTopX == 0:
            return ((bottomX,bottomY),(mid_X,(480-avgTopY)/2)) # was 480 but should be 240, but retuning required

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
        angle = np_arctan((CAR_CENTER_RATIO*640 - avgTopX)/(avgTopY-bottomY)) #was 640 but should be 424
        angle = 180*angle/np_pi
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
            #print(crossbox)
            avgY = 0
            for tar in crossbox:
                (X,Y) = tar[0]
                avgY = Y + avgY
            avgY = avgY/len(crossbox)
            #print(avgY)
            if avgY > 225:
                CROSSWALK = True

        if crossLines is not None:
            #print(crossLines)
            avgY = 0
            for tar in crossLines:
                (p1,p2) = tar
                #print(tar)
                #print(p1)
                (p1X,p1Y) = tuple(p1[0])
                (p2X,p2Y) = tuple(p2[0])
                avgY = avgY + p1Y + p2Y
            avgY = avgY/(len(crossLines)*2)
            #print(avgY)
            if avgY > 225:
                CROSSWALK = True

        # If There are any crosswalks return true 
        #if (crossbox is not None) or (crossLines is not None):
        #    CROSSWALK = True

        return CROSSWALK

    def runImageProcessing(self, laneDetect_routeManagerQ, stopDetect_routeManagerQ, emStopDetect_routeManagerQ,
                           yolo_pipe, yolo_ready_flag):
        pipe_output, pipe_input = yolo_pipe
        pipe_output.close()     # don't need to read from it

        self.poller = Pollers()
        self.emStopD.threshold = self.poller.getClippingDistance()
        print(" -- Set clipping distance to {} --".format(self.poller.getClippingDistance()))
        # print("starting delay for image processing")
        # for i in range(0,5):
        #     color, depth = self.poller.pollFrame()
        print("starting actual image processing")
        count = 0
        init_video()
        while True:
            color, depth = self.poller.pollFrame()
            # print("count: {}".format(count))
            if (color is None) and (depth is None):
                continue
            # cv_imwrite("testFrame{}.jpeg".format(count), color)

            angle = self.getCorrectionAngle(color)
            emStop = self.emStopD.detectStop(depth)
            stopLines = self.getCrosswalk(color)
            laneDetect_routeManagerQ.put(angle)
            stopDetect_routeManagerQ.put(stopLines) #stopLines)
            emStopDetect_routeManagerQ.put(emStop)
            count += 1

            if yolo_ready_flag.value >= 1:
                height = color.shape[0]
                width = color.shape[1]
                cropped_for_pipe = color[0:height//2, int(width/3):width-1]
                # print("Sending data through the pipe:\n\t\t{}".format(cropped_for_pipe.shape))
                pipe_input.send(cropped_for_pipe)
                yolo_ready_flag.value = 0


if __name__ == "__main__":
    print("Module is not runnable!")
