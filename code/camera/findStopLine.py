from os import (path as os_path,
                getcwd as os_getcwd,
                listdir as os_listdir)
from sys import path as sys_path
from math import ceil
from cv2 import (COLOR_GRAY2BGR as cv_COLOR_GRAY2BGR,
                COLOR_BGR2GRAY as cv_COLOR_BGR2GRAY,
                RETR_EXTERNAL as cv_RETR_EXTERNAL,
                CHAIN_APPROX_NONE as cv_CHAIN_APPROX_NONE,
                IMREAD_COLOR as cv_IMREAD_COLOR,
                ADAPTIVE_THRESH_MEAN_C as cv_ADAPTIVE_THRESH_MEAN_C,
                THRESH_BINARY as cv_THRESH_BINARY,
                MORPH_RECT as cv_MORPH_RECT,
                imread as cv_imread,
                line as cv_line,
                cvtColor as cv_cvtColor,
                inRange as cv_inRange,
                adaptiveThreshold as cv_adaptiveThreshold,
                warpPerspective as cv_warpPerspective,
                GaussianBlur as cv_GaussianBlur,
                HoughLinesP as cv_HoughLinesP,
                findContours as cv_findContours,
                drawContours as cv_drawContours,
                addWeighted as cv_addWeighted,
                Canny as cv_Canny,
                minAreaRect as cv_minAreaRect,
                boxPoints as cv_boxPoints,
                getStructuringElement as cv_getStructuringElement,
                erode as cv_erode,
                dilate as cv_dilate)
from numpy import ( pi as np_pi,
                    int0 as np_int0,
                    array as np_array,
                    asarray as np_asarray,
                    zeros_like as np_zeros_like,
                    copy as np_copy,
                    histogram as np_histogram,
                    mean as np_mean,
                    std as np_std)

sys_path.append(os_path.abspath(os_getcwd()))
from calibrate import getHomographyMatrix
from findLines import newColorSpace, displayImage, within
from contourPlus import contourPlus, warpPoints
from linePoint import linePoint


def cleanUpImage(img):
    # get the grayscale channel from HLS color space
    grayImg = newColorSpace(img, cNum=1)
    # threshold to only have white lines
    # generous threshold because of blurred images
    lowerBound = int(grayImg.max() - 40)
    threshWhite = cv_inRange(grayImg, lowerBound, 255)
    # displayImage("gray thresh", threshWhite)

    kSz = 7
    smoothed2 = cv_GaussianBlur(threshWhite, (kSz, kSz), 0)
    # displayImage("gray", smoothed2)
    return smoothed2


def findContours(img):
    canny = cv_Canny(img, 100, 200)

    cannyColor = cv_cvtColor(canny, cv_COLOR_GRAY2BGR)
    _, contours, hierarchy = cv_findContours(canny, cv_RETR_EXTERNAL, cv_CHAIN_APPROX_NONE)
    if len(contours) is 0:
        print("error length of contours is 0 (stp lines)")
        return None

    cv_drawContours(cannyColor, contours, -1, (0, 255, 0), 3)
    # displayImage("canny", cannyColor)

    contours2 = [contourPlus(x) for x in contours]
    contours2.sort(key=lambda x: x.getArea(), reverse=True)

    areas = [c.getArea() for c in contours2]
    meanArea = np_mean(areas)
    contours3 = [c for c in contours2 if c.getArea() > 2000]

    # get rid of anything with the wrong shape
    contours4 = [c for c in contours3 if c.getProportion() < 3]

    # sort by the center points
    centerSort = sorted(contours4, key=lambda x: x.center, reverse=False)
    # get rid of ones with much different areas
    stdevArea = np_std(areas)
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
    allPoints = np_array(allPoints)
    # print(allPoints)
    crosswalk = cv_minAreaRect(allPoints)
    crossbox = [np_int0(cv_boxPoints(crosswalk))]

    # display
    cv_drawContours(cannyColor, boxes, -1, (0, 0, 255), 7)
    cv_drawContours(cannyColor, crossbox, -1, (255, 0, 0), 7)
    # displayImage("contours", cannyColor)
    # black = np_zeros_like(canny)
    # cv_drawContours(black, approx, -1, 255, 5)
    # return black
    return crosswalk


def getHorizLines(img):
    # grayscale adaptive threshold
    threshed = cv_adaptiveThreshold(img, 255, cv_ADAPTIVE_THRESH_MEAN_C, cv_THRESH_BINARY, 15, -2)
    # displayImage("threshed", threshed)

    hSz = 17
    horizStructure = cv_getStructuringElement(cv_MORPH_RECT, (hSz, 1))

    # erode & dilate
    eroded = cv_erode(threshed, horizStructure, anchor=(-1, -1))
    dilated = cv_dilate(eroded, horizStructure, anchor=(-1, -1))

    # displayImage("eroded", dilated)
    return houghLines(dilated)


def houghLines(img):
    lines = cv_HoughLinesP(
        img, rho=1, theta=np_pi / 180,
        threshold=20, minLineLength=100, maxLineGap=20
    )
    if lines is None:
        return None

    color = [255, 0, 0]
    thickness = 1
    houghImg = np_copy(cv_cvtColor(img, cv_COLOR_GRAY2BGR))
    newLines = []

    # get rid of ones with bad slope
    for line in lines:
        for x1, y1, x2, y2 in line:
            # calculate the slope
            if (x2 - x1) != 0:
                slope = (y2-y1)/(x2-x1)
            else:
                slope = 1000000

            # throw out shallow lines
            if abs(slope) > 0.3:
                continue

            cv_line(houghImg, (x1, y1), (x2, y2), color, thickness)
            newLines.append([[x1, y1, x2, y2]])

    # displayImage("hough", houghImg)

    return newLines


def getAvgLines(lineSet):
    leftPoints = [l.p0 for l in lineSet]
    rightPoints = [l.p1 for l in lineSet]

    leftX, leftY = list(zip(*leftPoints))
    rightX, rightY = list(zip(*rightPoints))

    avgLeftX = int(np_mean(np_asarray(leftX)))
    avgLeftY = int(np_mean(np_asarray(leftY)))
    avgRightX = int(np_mean(np_asarray(rightX)))
    avgRightY = int(np_mean(np_asarray(rightY)))

    return linePoint([[avgLeftX, avgLeftY, avgRightX, avgRightY]])


def sortLines(img, lines):
    if lines is None:
        return None

    height = img.shape[0]
    width = img.shape[1]

    linePoints = [linePoint(l) for l in lines]
    yPoints = [l.p0[1] for l in linePoints]

    # find groups of lines
    binNum = height // 24  # most white lines are 20 or fewer pixels across in the y dimension
    hist = np_histogram(yPoints, bins=binNum)
    gap = ceil(hist[1][1] - hist[1][0])

    sortedLines = []
    for idx, binCnt in enumerate(hist[0]):
        if binCnt < 4:
            continue
        # find all the ones in this bin
        s = [linePoints[i] for i in range(len(yPoints)) if within(yPoints[i], hist[1][idx], gap)]
        sortedLines.append(s)

    if not sortedLines:
        return None

    # get the average lines
    avgLines = []
    for lineSet in sortedLines:
        if lineSet:
            avgLines.append(getAvgLines(lineSet))

    # reject ones that are too short
    crossLines = [l for l in avgLines if (l.p1[0] - l.p0[0]) > (width / 2)]
    if not crossLines:
        return None

    colorImg = cv_cvtColor(img, cv_COLOR_GRAY2BGR)
    for l in crossLines:
        cv_line(colorImg, l.p0, l.p1, (0, 0, 255), 4)

    # displayImage("avg lines", colorImg)
    return crossLines


def findStopLine(path, hmg, ihmg, debug=False):
    # input is string or image already loaded
    if isinstance(path, str):
        img = cv_imread(path, cv_IMREAD_COLOR)
    else:
        img = path

    kSz = 5
    warped = cv_warpPerspective(img, hmg, (img.shape[1], img.shape[0]))
    warped = cv_GaussianBlur(warped, (kSz, kSz), 0)
    # displayImage("warped", warped)

    # this is to find the crosswalks with boxes
    threshWhite = cleanUpImage(warped)
    crossbox = findContours(threshWhite)

    # this is to find the double line crosswalks
    gray = cv_cvtColor(warped, cv_COLOR_BGR2GRAY)
    lines = getHorizLines(gray)
    crossLines = sortLines(gray, lines)

    if crossbox is None:
        origBox = None
    else:
        # warp back to perspective
        boxPoints = cv_boxPoints(crossbox)
        origBox = warpPoints(boxPoints, ihmg)

    if not crossLines:
        origLines = None
    else:
        origLines = [warpPoints([[l.p0[0], l.p0[1]], [l.p1[0], l.p1[1]]], ihmg) for l in crossLines]

    if debug:
        return img, origBox, origLines
    else:
        return origBox, origLines


def drawCrossBox(img, box):
    pts = [(p[0][0], p[0][1]) for p in box]
    justBox = np_zeros_like(img)

    cv_line(justBox, pts[0], pts[1], (0, 255, 0), 5)
    cv_line(justBox, pts[1], pts[2], (0, 255, 0), 5)
    cv_line(justBox, pts[2], pts[3], (0, 255, 0), 5)
    cv_line(justBox, pts[3], pts[0], (0, 255, 0), 5)

    overlay = cv_addWeighted(img, 1.0, justBox, beta=0.95, gamma=0.0)
    return overlay


def drawCrossLines(img, lines):
    pts2 = [(tuple(p[0][0]), tuple(p[1][0])) for p in lines]
    justLines = np_zeros_like(img)

    for l in pts2:
        cv_line(justLines, l[0], l[1], (255, 0, 0), 5)

    overlay = cv_addWeighted(img, 1.0, justLines, beta=0.95, gamma=0.0)
    return overlay


def main():
    imgDir = os_path.abspath("../../testvideo/stopLines/frames")
    imgList = os_listdir(imgDir)
    imgs = sorted([os_path.join(imgDir, x) for x in imgList])

    hmg = getHomographyMatrix("color")
    invh = getHomographyMatrix("color", inverse=True)

    for i in imgs:
        if not "frame15" in i:
            continue
        print(os_path.basename(i))
        img, crossbox, crossLines = findStopLine(i, hmg, invh, debug=True)

        if crossbox is not None:
            img = drawCrossBox(img, crossbox)

        if crossLines:
            img = drawCrossLines(img, crossLines)

        displayImage("overlay", img)


if __name__ == '__main__':
    main()
