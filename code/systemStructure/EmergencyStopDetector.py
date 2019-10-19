import os
import sys
import cv2 as cv
from numpy import (arange,
                   uint8,
                   apply_along_axis)
sys.path.append(os.path.abspath("../camera"))
from findLines import displayImage

class EmergencyStopDetector(object):
    def __init__(self):
        pass

    def detectStop(self, frameQ, outputQ):
        """Detects if we should stop the car
        frameQ - multiprocessing queue which will contain depth frames
        outputQ - queue to which this will post the decision made
        """

        while True:
            try:
                # get the next frame from the queue
                depthFrame = frameQ.get()
                # check if we should stop
                result = self.parseFrame(depthFrame)
                # post the result to the output queue
                outputQ.put(result)

            except Exception as e:
                # print any exceptions that occur, but keep going
                print("Error in Emergency Stop Detector!")
                print(e)

    def maskImage(self, img):
        # width = img.shape[1]
        # height = img.shape[0]

        x0 = 150
        x1 = 520

        y0 = 150
        y1 = 380

        roi = img[y0:y1, x0:x1]
        return roi

    def checkForCloseObject(self, depthFrame):
        return depthFrame.max() > 200

    def parseFrame(self, depthFrame):
        """Returns True if something is ahead of it too close, False otherwise"""
        # displayImage("depth", depthFrame)

        roi = self.maskImage(depthFrame)
        displayImage("masked", roi)
        # https://github.com/IntelRealSense/librealsense/tree/development/wrappers/opencv/depth-filter

        return self.checkForCloseObject(roi)


def testMain():
    esd = EmergencyStopDetector()
    # imgDir = os.path.abspath("../../testimages/chessboard/depth")
    # imgName = "calibre_centerv1_1.jpeg"
    # imgs = [os.path.join(imgDir, imgName)]

    # imgDir = os.path.abspath("../../testimages/frames/depth")
    imgDir = os.path.abspath("../camera/tools/depthFrames")
    imgList = os.listdir(imgDir)
    imgs = sorted([os.path.join(imgDir, x) for x in imgList])

    for path in imgs:
        img = cv.imread(path)
        result = esd.parseFrame(img)
        print(result)


if __name__ == '__main__':
    testMain()
