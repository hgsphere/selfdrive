from os import (path as os_path,
                listdir as os_listdir)
from sys import (path as sys_path)
from cv2 import (imread as cv_imread)
from numpy import ( asanyarray as np_asanyarray,
                    where as np_where,
                    linspace as np_linspace)
sys_path.append(os_path.abspath("../camera"))
from findLines import displayImage


class EmergencyStopDetector(object):
    def __init__(self):
        self.pixel_threshold = 600#500
        self.count_threshold = 425#450

    def detectStop(self, frame):
        """Detects if we should stop the car
        frameQ - multiprocessing queue which will contain depth frames
        outputQ - queue to which this will post the decision made
        """

        # while True:
        try:
            # check if we should stop
            result = self.parseFrame(frame)
            return result
        except Exception as e:
            # print any exceptions that occur, but keep going
            print("Error in Emergency Stop Detector!")
            print(e)

    def maskImage(self, img): #424(x) 240(y)
        # width = img.shape[1]
        # height = img.shape[0]
        #print(img.shape)

        x0 = 160 #150
        x1 = 330 #340

        y0 = 120 #90
        y1 = 190 #210

        # roi = np_asanyarray(img[y0:y1, x0:x1], dtype="uint16")
        # rowNumbers = np_linspace(0, roi.shape[0]-1, num=5, dtype="uint16")
        # print(rowNumbers)
        # roiRowSample = roi[rowNumbers]

        # fast ROI calculation
        stripes = np_linspace(y0, y1, num=5, dtype="uint16")
        roi = np_asanyarray(img[stripes, x0:x1])

        # roi = img[y0:y1, x0:x1]
        # return roiRowSample
        return roi

    def checkForCloseObject(self, depthFrame):
        #print(depthFrame.shape)
        count = len((np_where(depthFrame < self.pixel_threshold))[0])
        #print(count)
        print("num values above threshold: {}".format(count))
        # print("min value = {}".format(depthFrame.min()))
        # return depthFrame.max() < self.threshold
        return count > self.count_threshold

    def parseFrame(self, depthFrame):
        """Returns True if something is ahead of it too close, False otherwise"""
        # displayImage("depth", depthFrame)

        roi = self.maskImage(depthFrame)
        # displayImage("masked", roi)
        # https://github.com/IntelRealSense/librealsense/tree/development/wrappers/opencv/depth-filter

        return self.checkForCloseObject(roi)


def testMain():
    esd = EmergencyStopDetector()
    # imgDir = os_path.abspath("../../testimages/chessboard/depth")
    # imgName = "calibre_centerv1_1.jpeg"
    # imgs = [os_path.join(imgDir, imgName)]

    # imgDir = os_path.abspath("../../testimages/frames/depth")
    imgDir = os_path.abspath("../camera/tools/depthFrames")
    imgList = os_listdir(imgDir)
    imgs = sorted([os_path.join(imgDir, x) for x in imgList])

    for path in imgs:
        img = cv_imread(path)
        result = esd.parseFrame(img)
        print(result)


if __name__ == '__main__':
    testMain()
