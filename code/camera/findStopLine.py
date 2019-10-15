import os
import sys
import cv2 as cv
import numpy as np
import numpy.polynomial.polynomial as poly

sys.path.append(os.path.abspath(os.getcwd()))
from calibrate import getHomographyMatrix
from findLines import newColorSpace, displayImage, within
from contourPlus import contourPlus, warpPoints


def cleanUpImage(img):
    # get the grayscale channel from HLS color space
    grayImg = newColorSpace(img, cNum=1)
    # threshold to only have white lines
    lowerBound = int(grayImg.max() - 30)
    threshWhite = cv.inRange(grayImg, lowerBound, 255)
    # displayImage("gray thresh", threshWhite)

    kSz = 7
    smoothed2 = cv.GaussianBlur(threshWhite, (kSz, kSz), 0)
    # displayImage("gray", smoothed2)
    return smoothed2


def findContours(img):
    canny = cv.Canny(img, 100, 200)

    cannyColor = cv.cvtColor(canny, cv.COLOR_GRAY2BGR)
    contours, hierarchy = cv.findContours(canny, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    cv.drawContours(cannyColor, contours, -1, (0, 255, 0), 3)
    # displayImage("canny", cannyColor)

    contours2 = [contourPlus(x) for x in contours]
    contours2.sort(key=lambda x: x.getArea(), reverse=True)

    areas = [c.getArea() for c in contours2]
    meanArea = np.mean(areas)
    # if len(contours2) > 4:
    #     # accept anything above the mean
    #     contours3 = [c for c in contours2 if c.getArea() > mean]
    # else:
    contours3 = [c for c in contours2 if c.getArea() > 2000]

    # get rid of anything with the wrong shape
    contours4 = [c for c in contours3 if c.getProportion() < 3]

    # sort by the center points
    centerSort = sorted(contours4, key=lambda x: x.center, reverse=False)
    # get rid of ones with much different areas
    stdevArea = np.std(areas)
    centerSort2 = [c for c in centerSort if within(c.getArea(), meanArea, stdevArea*2)]

    # we need to find at least 3
    if len(centerSort2) < 3:
        return None

    # polygon approximation
    boxes = [c.getBoxAsContour() for c in centerSort2]
    # approx = [c.approx for c in centerSort2]

    # combine it all into one bounding box
    allPoints = []
    for b in boxes:
        for p in b:
            allPoints.append(p)
    allPoints = np.array(allPoints)
    # print(allPoints)
    crosswalk = cv.minAreaRect(allPoints)
    crossbox = [np.int0(cv.boxPoints(crosswalk))]

    # display
    cv.drawContours(cannyColor, boxes, -1, (0, 0, 255), 7)
    cv.drawContours(cannyColor, crossbox, -1, (255, 0, 0), 7)
    # displayImage("contours", cannyColor)
    # black = np.zeros_like(canny)
    # cv.drawContours(black, approx, -1, 255, 5)
    # return black
    return crosswalk


def findStopLine(path, hmg, ihmg, debug=False):
    # input is string or image already loaded
    if isinstance(path, str):
        img = cv.imread(path, cv.IMREAD_COLOR)
    else:
        img = path

    kSz = 5
    warped = cv.warpPerspective(img, hmg, (img.shape[1], img.shape[0]))
    warped = cv.GaussianBlur(warped, (kSz, kSz), 0)

    threshWhite = cleanUpImage(warped)
    crossbox = findContours(threshWhite)

    if crossbox is None:
        origBox = None
    else:
        # warp back to perspective
        boxPoints = cv.boxPoints(crossbox)
        origBox = warpPoints(boxPoints, ihmg)

    if debug:
        return img, origBox
    else:
        return origBox


def main():
    imgDir = os.path.abspath("../../testvideo/stopLines/frames")
    imgList = os.listdir(imgDir)
    imgs = sorted([os.path.join(imgDir, x) for x in imgList])

    hmg = getHomographyMatrix("color")
    invh = getHomographyMatrix("color", inverse=True)

    for i in imgs:
        print(os.path.basename(i))
        img, crossbox = findStopLine(i, hmg, invh, debug=True)

        if crossbox is None:
            continue

        pts = [(p[0][0], p[0][1]) for p in crossbox]
        cv.line(img, pts[0], pts[1], (0, 255, 0), 5)
        cv.line(img, pts[1], pts[2], (0, 255, 0), 5)
        cv.line(img, pts[2], pts[3], (0, 255, 0), 5)
        cv.line(img, pts[3], pts[0], (0, 255, 0), 5)

        displayImage("overlay", img)


if __name__ == '__main__':
    main()
