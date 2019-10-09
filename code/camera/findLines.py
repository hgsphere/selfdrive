import os
import sys
import imutils
import cv2 as cv
import numpy as np
from pprint import PrettyPrinter

sys.path.append(os.path.abspath(os.getcwd()))
from calibrate import getHomographyMatrix


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
    threshWhite = cv.inRange(grayImg, 115, 255)
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
            slope = (y2-y1)/(x2-x1)

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


def getLinesPoints(img, lines):
    height = img.shape[0]
    width = img.shape[1]
    img2 = np.zeros_like(img)

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
    hist, bins = np.histogram(x_intercepts)

    histSort = hist.argsort()[-2:]
    binLanes = sorted([int(bins[x]) for x in histSort])
    lanes = []

    for l in binLanes:
        slope = [x for x in slopes if within(x[1], l, 20)]
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
    return lanes


def getContours(canny):
    cannyColor = cv.cvtColor(canny, cv.COLOR_GRAY2BGR)
    contours, hierarchy = cv.findContours(canny, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    # remove contours with small area

    # contours = sorted(contours, key=cv.contourArea)
    # cv.drawContours(cannyColor, contours, -1, (0, 255, 0), -1)
    rank = np.zeros((len(contours)))
    for i in range(0, len(contours)):
        rank[i] = contours[i].shape[0]
    top = rank.argsort()
    # displayImage("contours", cannyColor)

    # return

    # minAreaRect returns tuple of ((centerX, centerY), (width, height), angle)
    approx = []
    end = len(contours) if len(contours) < 4 else 4
    for i in top[-end:]:
        c = contours[top[i]]
        app = cv.approxPolyDP(c, epsilon=0.1, closed=True)

        rect = cv.minAreaRect(c)
        rSize = rect[1]
        rWidth, rHeight = rSize[0], rSize[1]
        if (rWidth == 0) or (rHeight == 0):
            continue
        rProportion = max((rWidth / rHeight), (rHeight / rWidth))
        # rArea = rSize[0] * rSize[1]
        # # filter by area
        # if rArea < 200:
        #     continue
        # filter by shape
        if rProportion < 2:
            continue
        approx.append(app)
        # print(rArea, rProportion)
        # box = cv.boxPoints(rect)
        # box = np.int0(box)
        # cv.drawContours(cannyColor, [box], 0, (0, 0, 255), 2)

    cv.drawContours(cannyColor, approx, -1, (0, 255, 0), 10)
    # displayImage("contours", cannyColor)
    black = np.zeros_like(canny)
    cv.drawContours(black, approx, -1, 255, 5)
    return black


def getHeading(leftP, rightP, img):
    width = img.shape[1]
    height = img.shape[0]

    # average of top x value of points
    avgTopX = (leftP[1][0] + rightP[1][0]) // 2
    avgTopY = 0

    # bottomX = width // 2
    bottomX = 291   # actually?
    bottomY = height

    # line equation
    m = (avgTopY - bottomY) / (avgTopX - bottomX)
    b = bottomY - m*bottomX

    # make sure doesn't exceed image boundaries
    if avgTopX < 0:
        avgTopY = int(b)
        avgTopX = 0
    elif avgTopX > width:
        avgTopY = int(m*width + b)
        avgTopX = width

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
    cv.waitKey(0)


def printContour(img, con, i):
    M = cv.moments(con)
    try:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    except ZeroDivisionError as e:
        return

    cv.putText(img, "#{}".format(i), (cX-20, cY), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 255), 1)
    return


def parseImage(path, hmg, invh):
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

    try:
        # apply Canny edge detection
        canny = cv.Canny(smoothed, 100, 200)
        # displayImage("Canny", canny)
        contourImg = getContours(canny)
        # contourImg = cv.cvtColor(cannyColor, cv.COLOR_BGR2GRAY)

        lines = houghLines(contourImg)
        leftPoints, rightPoints = getLinesPoints(canny, lines)

        # average middle line
        target = getHeading(leftPoints, rightPoints, warped)
        overlay = showHeading(target, warped, img, invh)

        return overlay
    except Exception as e:
        return img


def main():
    imgDir = os.path.abspath("../../testimages/lanes/rgb")
    imgList = os.listdir(imgDir)
    imgs = sorted([os.path.join(imgDir, x) for x in imgList])

    hmg = getHomographyMatrix("color")
    invh = getHomographyMatrix("color", inverse=True)

    for i in imgs:
        parseImage(i, hmg, invh)


if __name__ == '__main__':
    main()