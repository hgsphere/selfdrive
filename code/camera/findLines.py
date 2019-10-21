import os
import sys
import cv2 as cv
import numpy as np
from pprint import PrettyPrinter

sys.path.append(os.path.abspath(os.getcwd()))
from calibrate import getHomographyMatrix
from contourPlus import contourPlus, warpPoints


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
    threshYellow = cv.inRange(satImg, 150, 255)
    # apply a Gaussian blur to smooth edges and remove noise
    kSz = 7
    smoothed = cv.GaussianBlur(threshYellow, (kSz, kSz), 0)
    # displayImage("yellow thresh", smoothed)

    # get yellow from hsv
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    lower_yellow = np.array([10, 100, 100], dtype=np.uint8)
    upper_yellow = np.array([40, 255, 255], dtype=np.uint8)
    mask_yellow = cv.inRange(hsv, lower_yellow, upper_yellow)
    blur_yellow = cv.GaussianBlur(mask_yellow, (kSz, kSz), 0)
    # displayImage("mask_yellow", mask_yellow)

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
    combined = cv.bitwise_or(combined, blur_yellow)
    return combined


def houghLines(img):
    lines = cv.HoughLinesP(
        img, rho=1, theta=np.pi / 180,
        threshold=20, minLineLength=20, maxLineGap=200
    )
    if lines is None or len(lines) == 0:
        return None

    color = [255, 0, 0]
    thickness = 1
    houghImg = np.copy(cv.cvtColor(img, cv.COLOR_GRAY2BGR))
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
    weights = []

    for l in binLanes:
        slope = [x for x in slopes if within(x[1], l, withinVal)]
        if not slope:
            slope = [x for x in slopes if within(x[1], l, withinVal*2)]
            if not slope:
                return None
        weights.append(len(slope))
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

    # print(lanes)
    # displayImage("lanes", imgColor)
    if debug:
        return lanes, imgColor
    else:
        return lanes


def getContours(canny):
    cannyColor = cv.cvtColor(canny, cv.COLOR_GRAY2BGR)
    if cv.getVersionMajor() == 3:
        _, contours, hierarchy = cv.findContours(canny, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    else:
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
    if len(contours2) > 4:
        # accept anything above the mean
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
    if (avgTopX - bottomX) != 0:
        m = (avgTopY - bottomY) / (avgTopX - bottomX)
    else:
        m = 1000000
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


# doesn't do any transformations
def showHeading(line, orig):
    if line is None:
        return orig

    p0, p1 = line

    justLine = np.zeros_like(orig)
    cv.line(justLine, p0, p1, (0, 0, 255), 3)

    overlay = cv.addWeighted(orig, 1.0, justLine, beta=0.95, gamma=0.0)
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
        print("invalid quadrant!")
        #print("invalid quadrant!", file=sys.stderr) there was some error here
        return None

    # subsample image
    interp = cv.INTER_AREA
    smallImg = cv.resize(img, (halfW, halfH), interpolation=interp)

    # put on image
    sys.stdout.flush()
    bigImg[p0[1]:p1[1], p0[0]:p1[0]] = smallImg

    return bigImg


# example input of too close
# [[(505, 480), (147, 0)], [(518, 480), (283, 0)]]
def fixLaneData(img, leftLane, rightLane):
    width = img.shape[1]
    withinVal = width // 12
    midPt = width // 2
    # estimates for average lane width
    # bottomLaneWidth = 250
    # topLaneWidth = 350
    bottomLaneWidth = 150
    topLaneWidth = 150
    # unpack
    lP0, lP1 = leftLane
    rP0, rP1 = rightLane

    # if they are the same thing, or too close together,
    #  then we need to invent the other side by estimating
    #  the width of the lane

    if leftLane == rightLane:
        pass
    elif within(lP0[0], rP0[0], withinVal):
        pass
    else:
        return leftLane, rightLane

    # closer to which side?
    if lP0[0] > midPt:
        # create new left lane
        leftLane = [(rP0[0]-bottomLaneWidth, rP0[1]), (rP1[0]-topLaneWidth, rP1[1])]
        # print("new left lane")
    elif rP0[0] < midPt:
        # create new right lane
        rightLane = [(lP0[0] + bottomLaneWidth, lP0[1]), (lP1[0] + topLaneWidth, lP1[1])]
        # print("new right lane")

    return leftLane, rightLane


def parseImage(path, hmg, invh, debug=False):
    """parseImage
    returns the line that describes the target to follow
    if debug is True, returns tuple that contains target and collage image
    """

    # allow passing the image in directly
    if isinstance(path, str):
        img = cv.imread(path, cv.IMREAD_COLOR)
    else:
        img = path
    kSz = 5
    # displayImage('input',img)
    # element = cv.getStructuringElement(cv.MORPH_RECT, (kSz, kSz), (-1, -1))
    # img = cv.morphologyEx(img, cv.MORPH_OPEN, element)
    warped = cv.warpPerspective(img, hmg, (img.shape[1], img.shape[0]))
    warped = cv.GaussianBlur(warped, (kSz, kSz), 0)
    # kill the top of the image
    warped = cv.rectangle(warped, (0, 0), (img.shape[1], img.shape[0] // 2), (0, 0, 0), -1)
    # displayImage("warped", warped)

    # convert color space, threshold the image, remove noise
    smoothed = cleanupImage(warped)
#    smoothed = cleaenupImage(img)
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
        if not lines:
            print('no lines found')
            return None
        points = getLinesPoints(canny, lines, debug=debug)
        if not points:
            print('no points found')
            return None
        if debug:
            points, laneImg = points
            leftPoints, rightPoints = points
        else:
            leftPoints, rightPoints = points

        # print("original lanes: {}, {}".format(leftPoints, rightPoints))
        leftLane, rightLane = fixLaneData(canny, leftPoints, rightPoints)
        # print("new lanes: {}, {}".format(leftLane, rightLane))
        # sys.stdout.flush()

        # average middle line
        target = getHeading(leftLane, rightLane, warped)
        ot = warpPoints(target, invh)
        # restructure
        origTarget = ((ot[0][0][0], ot[0][0][1]), (ot[1][0][0], ot[1][0][1]))
        # print(origTarget)

        if debug:
            overlay = showHeading(origTarget, img)
            collage = addImageQuadrant(collage, overlay, 0)
            collage = addImageQuadrant(collage, contourImg, 2)
            collage = addImageQuadrant(collage, laneImg, 3)
            return origTarget, collage
        else:
            return origTarget

    except Exception as e:
        raise e
        if not debug:
            return None
        else:
            # print(e)
            return None, collage


def main():
    imgDir = os.path.abspath("../../testimages/lowres")
    # imgDir = os.path.abspath("../../testvideo/frames")
    imgList = os.listdir(imgDir)
    imgs = sorted([os.path.join(imgDir, x) for x in imgList])

    hmg = getHomographyMatrix("color-lowres")
    invh = getHomographyMatrix("color-lowres", inverse=True)

    for i in imgs:
        # if not "frame9" in i:
        #     continue
        print(os.path.basename(i))
        result = parseImage(i, hmg, invh, debug=True)
        if result is not None:
            target, overlay = result
            displayImage("overlay", overlay)


if __name__ == '__main__':
    main()
