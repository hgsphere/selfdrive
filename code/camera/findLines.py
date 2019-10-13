import os
import sys
import cv2 as cv
import numpy as np
from pprint import PrettyPrinter

sys.path.append(os.path.abspath(os.getcwd()))
from calibrate import getHomographyMatrix
from contourPlus import contourPlus


def newColorSpace(img, cNum=2):
    hls = cv.cvtColor(img, cv.COLOR_BGR2HLS)
    chan = cv.extractChannel(hls, cNum)

    # we want to look at the saturation channel
    return chan


def cleanupImage(img):
    # get the saturation channel from HLS color space
    satImg = newColorSpace(img)
    # displayImage("saturation", satImg)
    # the yellow looks white and the white looks black

    # threshold to only keep yellow lines
    threshYellow = cv.inRange(satImg, 220, 255)
    # apply a Gaussian blur to smooth edges and remove noise
    kSz = 7
    smoothed = cv.GaussianBlur(threshYellow, (kSz, kSz), 0)
    # displayImage("yellow thresh", smoothed)

    # get the grayscale channel from HLS color space
    grayImg = newColorSpace(img, cNum=1)
    # displayImage("grayscale", grayImg)
    # threshold to only have white lines
    lowerBound = int(grayImg.max() - 30)
    threshWhite = cv.inRange(grayImg, lowerBound, 255)
    # displayImage("gray thresh", threshWhite)
    smoothed2 = cv.GaussianBlur(threshWhite, (kSz, kSz), 0)

    # mask the 2 images together
    combined = cv.bitwise_or(smoothed, smoothed2)
    return combined


def houghLines(img):
    lines = cv.HoughLinesP(
        img, rho=1, theta=np.pi / 180,
        threshold=20, minLineLength=20, maxLineGap=200
    )

    color = [255, 0, 0]
    thickness = 1
    houghImg = np.copy(cv.cvtColor(img, cv.COLOR_GRAY2BGR))
    newLines = []

    # get rid of ones with bad slope
    for line in lines:
        for x1, y1, x2, y2 in line:
            # calculate the slope
            try:
                slope = (y2-y1)/(x2-x1)
            except RuntimeWarning:
                # divide by 0 error
                # TODO: I don't think check is working, but it doesn't crash
                slope = 1000000

            # throw out shallow lines
            if abs(slope) < 0.3:
                continue

            cv.line(houghImg, (x1, y1), (x2, y2), color, thickness)
            newLines.append([[x1, y1, x2, y2]])

    # displayImage("hough", houghImg)

    return newLines


def within(x, target, threshold):
    if abs(x - target) < threshold:
        return True
    else:
        return False


def getLinesPoints(img, lines, debug=False):
    height = img.shape[0]
    width = img.shape[1]

    # sort by x-intercept
    slopes = []
    for line in lines:
        for x1, y1, x2, y2 in line:
            if (x2-x1) != 0:
                m = (y2-y1)/(x2-x1)
            else:
                m = 1000000

            b = y1 - m*x1
            xInt = (height - b) / m
            slopes.append([m, xInt])

    pp = PrettyPrinter()
    # pp.pprint(slopes)

    # histogram of x-intercepts
    x_intercepts = list(zip(*slopes))[1]
    # pp.pprint(x_intercepts)
    binNum = 12
    withinVal = width // binNum
    hist, bins = np.histogram(x_intercepts, bins=12)

    histSort = hist.argsort()[-2:]
    binLanes = sorted([int(bins[x]) for x in histSort])
    lanes = []

    for l in binLanes:
        slope = [x for x in slopes if within(x[1], l, withinVal)]
        slopeList, intcList = list(zip(*slope))
        avgSlope = np.mean(np.asarray(slopeList))
        avgInt = int(np.mean(np.asarray(intcList)))
        p0 = (avgInt, height)
        p1 = (avgInt - int(height / avgSlope), 0)
        lanes.append([p0, p1])

    imgColor = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    GREEN = (0, 255, 0)

    for p0, p1 in lanes:
        cv.line(imgColor, p0, p1, GREEN, 3)

    # displayImage("lanes", imgColor)
    if debug:
        return lanes, imgColor
    else:
        return lanes


def getContours(canny):
    cannyColor = cv.cvtColor(canny, cv.COLOR_GRAY2BGR)
    contours, hierarchy = cv.findContours(canny, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    # get extra data about the contours
    contours2 = [contourPlus(x) for x in contours]
    # sort by the area
    contours2.sort(key=lambda x: x.getArea(), reverse=True)
    # for c in contours2:
    #     print(c.getArea())
    # get the average
    mean = np.mean([c.getArea() for c in contours2])
    # sd = np.std([c.getArea() for c in contours2])
    # accept anything 1 standard deviation above the mean
    if len(contours2) > 4:
        contours3 = [c for c in contours2 if c.getArea() > mean]
    else:
        contours3 = [c for c in contours2 if c.getArea() > 200]
    # get rid of anything with the wrong shape
    contours4 = [c for c in contours3 if c.getProportion() > 2]
    # polygon approximation
    approx = [c.approx for c in contours4]

    # display
    cv.drawContours(cannyColor, approx, -1, (0, 255, 0), 10)
    # displayImage("contours", cannyColor)
    black = np.zeros_like(canny)
    cv.drawContours(black, approx, -1, 255, 5)
    return black


def getHeading(leftPts, rightPts, img):
    width = img.shape[1]
    height = img.shape[0]

    # average of top x value of points
    avgTopX = (leftPts[1][0] + rightPts[1][0]) // 2
    avgTopY = 0

    # bottomX = width // 2
    # bottomX = 291   # according to transform
    bottomX = (leftPts[0][0] + rightPts[0][0]) // 2
    bottomY = height

    # line equation
    m = (avgTopY - bottomY) / (avgTopX - bottomX)
    b = bottomY - m*bottomX

    # make sure doesn't exceed image boundaries
    if avgTopX < 0:
        avgTopX = 0
        avgTopY = int(b)
    elif avgTopX > width:
        avgTopX = width
        avgTopY = int(m*width + b)

    if bottomX < 0:
        bottomX = 0
        bottomY = int(b)
    elif bottomX > width:
        bottomX = width
        bottomY = int(m*width + b)

    p0 = (bottomX, bottomY)
    p1 = (avgTopX, avgTopY)

    # return 2 points
    return p0, p1


def showHeading(line, img, orig, hmg):
    p0, p1 = line

    cv.line(img, p0, p1, (0, 0, 255), 3)
    # displayImage("heading", img)

    justLine = np.zeros_like(img)
    cv.line(justLine, p0, p1, (0, 0, 255), 3)

    fixed = cv.warpPerspective(justLine, hmg, (img.shape[1], img.shape[0]))
    # displayImage("fixed", fixed)

    overlay = cv.addWeighted(orig, 1.0, fixed, beta=0.95, gamma=0.0)
    # displayImage("overlay", overlay)

    return overlay


def displayImage(name, mat):
    cv.imshow(name, mat)
    return cv.waitKey(0)


def addImageQuadrant(bigImg, img, quadrant):
    """addImageQuadrant
    -------------
    |  0  |  1  |
    -------------
    |  2  |  3  |
    -------------
    """

    # color convert
    if len(img.shape) == 2:
        img = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    # we know they are all 640 x 480
    width = img.shape[1]
    height = img.shape[0]
    halfW = width // 2
    halfH = height // 2

    if quadrant == 0:
        p0 = (0, 0)
        p1 = (halfW, halfH)
    elif quadrant == 1:
        p0 = (halfW, 0)
        p1 = (width, halfH)
    elif quadrant == 2:
        p0 = (0, halfH)
        p1 = (halfW, height)
    elif quadrant == 3:
        p0 = (halfW, halfH)
        p1 = (width, height)
    else:
        print("invalid quadrant!", file=sys.stderr)
        return None

    # subsample image
    interp = cv.INTER_AREA
    smallImg = cv.resize(img, (halfW, halfH), interpolation=interp)

    # put on image
    sys.stdout.flush()
    bigImg[p0[1]:p1[1], p0[0]:p1[0]] = smallImg

    return bigImg


def parseImage(path, hmg, invh, debug=False):
    """parseImage"""

    # allow passing the image in directly
    if isinstance(path, str):
        img = cv.imread(path, cv.IMREAD_COLOR)
    else:
        img = path
    kSz = 5
    # element = cv.getStructuringElement(cv.MORPH_RECT, (kSz, kSz), (-1, -1))
    # img = cv.morphologyEx(img, cv.MORPH_OPEN, element)
    warped = cv.warpPerspective(img, hmg, (img.shape[1], img.shape[0]))
    warped = cv.GaussianBlur(warped, (kSz, kSz), 0)
    # kill the top of the image
    warped = cv.rectangle(warped, (0, 0), (img.shape[1], img.shape[0] // 2), (0, 0, 0), -1)
    # displayImage("eroded", warped)

    # convert color space, threshold the image, remove noise
    smoothed = cleanupImage(warped)
    # displayImage("smoothed", smoothed)

    # collage of images - original (w/ heading), smoothed, contours & canny, lanes
    if debug:
        collage = np.zeros_like(img)
        collage = addImageQuadrant(collage, img, 0)
        collage = addImageQuadrant(collage, smoothed, 1)

    try:
        # apply Canny edge detection
        canny = cv.Canny(smoothed, 100, 200)
        # displayImage("Canny", canny)
        contourImg = getContours(canny)
        # contourImg = cv.cvtColor(cannyColor, cv.COLOR_BGR2GRAY)

        lines = houghLines(contourImg)
        points = getLinesPoints(canny, lines, debug=debug)
        if debug:
            points, laneImg = points
            leftPoints, rightPoints = points
        else:
            leftPoints, rightPoints = points

        # average middle line
        target = getHeading(leftPoints, rightPoints, warped)
        overlay = showHeading(target, warped, img, invh)

        if debug:
            collage = addImageQuadrant(collage, overlay, 0)
            collage = addImageQuadrant(collage, contourImg, 2)
            collage = addImageQuadrant(collage, laneImg, 3)
            return collage
        else:
            return overlay

    except Exception as e:
        if not debug:
            return img
        else:
            # print(e)
            return collage


def main():
    imgDir = os.path.abspath("../../testimages/frames/rgb")
    imgList = os.listdir(imgDir)
    imgs = sorted([os.path.join(imgDir, x) for x in imgList])

    hmg = getHomographyMatrix("color")
    invh = getHomographyMatrix("color", inverse=True)

    for i in imgs:
        overlay = parseImage(i, hmg, invh, debug=True)
        print(os.path.basename(i))
        displayImage("overlay", overlay)


if __name__ == '__main__':
    main()