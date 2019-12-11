from os import (path as os_path,
                getcwd as os_getcwd,
                listdir as os_listdir)
from sys import (path as sys_path,
                stdout as sys_stdout)
from cv2 import (COLOR_GRAY2BGR as cv_COLOR_GRAY2BGR,
                COLOR_BGR2HLS as cv_COLOR_BGR2HLS,
                COLOR_BGR2HSV as cv_COLOR_BGR2HSV,
                RETR_EXTERNAL as cv_RETR_EXTERNAL,
                CHAIN_APPROX_NONE as cv_CHAIN_APPROX_NONE,
                INTER_AREA as cv_INTER_AREA,
                IMREAD_COLOR as cv_IMREAD_COLOR,
                imread as cv_imread,
                imwrite as cv_imwrite,
                imshow as cv_imshow,
                waitKey as cv_waitKey,
                VideoWriter_fourcc as cv_VideoWriter_fourcc,
                VideoWriter as cv_VideoWriter,
                line as cv_line,
                rectangle as cv_rectangle,
                cvtColor as cv_cvtColor,
                extractChannel as cv_extractChannel,
                inRange as cv_inRange,
                bitwise_or as cv_bitwise_or,
                resize as cv_resize,
                warpPerspective as cv_warpPerspective,
                GaussianBlur as cv_GaussianBlur,
                HoughLinesP as cv_HoughLinesP,
                getVersionMajor as cv_getVersionMajor,
                findContours as cv_findContours,
                drawContours as cv_drawContours,
                addWeighted as cv_addWeighted,
                Canny as cv_Canny)
from numpy import (array as np_array,
                    zeros_like as np_zeros_like,
                    uint8 as np_uint8,
                    pi as np_pi,
                    copy as np_copy,
                    histogram as np_histogram,
                    mean as np_mean,
                    asarray as np_asarray,
                    std as np_std)

sys_path.append(os_path.abspath(os_getcwd()))
from calibrate import getHomographyMatrix
from contourPlus import contourPlus, warpPoints

printOnce = 0

## ADDED ability to save video
SAVE_VIDEO = False

writer_rbg = None
def init_video():
    global writer_rgb
    shape_rgb = (424, 240)
    frame_rate_rgb = 30
    fourcc = cv_VideoWriter_fourcc(*"MJPG")
    outpath_rgb = os_path.join(os_getcwd(), "logVideo.avi")
    #outpath_dep = os_path.join(os_getcwd(), "output-depth-low.avi")
    writer_rgb = cv_VideoWriter(outpath_rgb, fourcc, frame_rate_rgb,
        (shape_rgb[0], shape_rgb[1]), True)



def newColorSpace(img, cNum=2):
    hls = cv_cvtColor(img, cv_COLOR_BGR2HLS)
    chan = cv_extractChannel(hls, cNum)

    # we want to look at the saturation channel
    return chan


def cleanupImage(img):
    # get the saturation channel from HLS color space
    satImg = newColorSpace(img)
    # displayImage("saturation", satImg)
    # the yellow looks white and the white looks black

    # threshold to only keep yellow lines
    threshYellow = cv_inRange(satImg, 150, 255)
    # displayImage("threshYellow", threshYellow)
    # apply a Gaussian blur to smooth edges and remove noise
    kSz = 7
    smoothed = cv_GaussianBlur(threshYellow, (kSz, kSz), 0)
    # kill the part of the image that has the bumper in it
    cv_rectangle(smoothed, (160, 200), (240, 239), color=0, thickness=-1)
    smoothed = np_zeros_like(satImg)
    # displayImage("yellow thresh", smoothed)

    # get yellow from hsv
    hsv = cv_cvtColor(img, cv_COLOR_BGR2HSV)
    lower_yellow = np_array([10, 100, 100], dtype=np_uint8)
    upper_yellow = np_array([40, 255, 255], dtype=np_uint8)
    mask_yellow = cv_inRange(hsv, lower_yellow, upper_yellow)
    blur_yellow = cv_GaussianBlur(mask_yellow, (kSz, kSz), 0)
    # displayImage("mask_yellow", blur_yellow)

    # get the grayscale channel from HLS color space
    grayImg = newColorSpace(img, cNum=1)
    # displayImage("grayscale", grayImg)
    # threshold to only have white lines
    lowerBound = int(grayImg.max() - 30)
    threshWhite = cv_inRange(grayImg, lowerBound, 255)
    # kill the part of the image that has detected the bumper
    cv_rectangle(threshWhite, (165, 210), (245, 230), color=0, thickness=-1)

    # displayImage("gray thresh", threshWhite)
    smoothed2 = cv_GaussianBlur(threshWhite, (kSz, kSz), 0)

    # mask the 2 yellow images together
    combined = cv_bitwise_or(smoothed, blur_yellow)

    # displayImage("combined", combined)
    return combined, smoothed2
    # returns yellow, white


def houghLines(img):
    lines = cv_HoughLinesP(
        img, rho=1, theta=np_pi / 180,
        threshold=20, minLineLength=20, maxLineGap=200
    )
    if lines is None or len(lines) == 0:
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
            if abs(slope) < 0.3:
                continue

            cv_line(houghImg, (x1, y1), (x2, y2), color, thickness)
            newLines.append([[x1, y1, x2, y2]])

    # displayImage("hough", houghImg)

    return newLines


def within(x, target, threshold):
    if abs(x - target) < threshold:
        return True
    else:
        return False


def getLinesPoints(img, lines, debug=False, lineCnt=2):
    if lines is None:
        if debug:
            return [None, None], cv_cvtColor(img, cv_COLOR_GRAY2BGR)
        else:
            return [None, None]

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

    if len(slopes) == 0:
        if debug:
            return [None, None], cv_cvtColor(img, cv_COLOR_GRAY2BGR)
        else:
            return [None, None]

    # histogram of x-intercepts
    x_intercepts = list(zip(*slopes))[1]
    # pp.pprint(x_intercepts)
    binNum = 12
    withinVal = width // binNum
    hist, bins = np_histogram(x_intercepts, bins=12)

    histSort = hist.argsort()[-lineCnt:]
    binLanes = sorted([int(bins[x]) for x in histSort])
    lanes = []
    weights = []

    for l in binLanes:
        slope = [x for x in slopes if within(x[1], l, withinVal)]
        if not slope:
            slope = [x for x in slopes if within(x[1], l, withinVal*2)]
            if not slope:
                return [None, None]
        weights.append(len(slope))
        slopeList, intcList = list(zip(*slope))

        avgSlope = np_mean(np_asarray(slopeList))
        avgInt = int(np_mean(np_asarray(intcList)))

        p0 = (avgInt, height)
        if avgSlope == 0:
            p1 = (avgInt,0)
        else:
            p1 = (avgInt - int(height / avgSlope), 0)
        lanes.append([p0, p1])

    imgColor = cv_cvtColor(img, cv_COLOR_GRAY2BGR)
    GREEN = (0, 255, 0)

    for p0, p1 in lanes:
        # OverflowError: signed integer is greater than maximum ###################### TODO
        # Im just gonna skip this for now
        pass
        cv_line(imgColor, p0, p1, GREEN, 3)

    # displayImage("lanes", imgColor)
    if debug:
        return lanes, imgColor
    else:
        return lanes


def getContours(canny):
    cannyColor = cv_cvtColor(canny, cv_COLOR_GRAY2BGR)
    if cv_getVersionMajor() == 3:
        _, contours, hierarchy = cv_findContours(canny, cv_RETR_EXTERNAL, cv_CHAIN_APPROX_NONE)
    else:
        contours, hierarchy = cv_findContours(canny, cv_RETR_EXTERNAL, cv_CHAIN_APPROX_NONE)
    if len(contours) is 0:
        print("error contours has length 0")
        return None

    # get extra data about the contours
    contours2 = [contourPlus(x) for x in contours]
    # sort by the area
    contours2.sort(key=lambda x: x.getArea(), reverse=True)
    # get the average
    mean = np_mean([c.getArea() for c in contours2])

    # sd = np_std([c.getArea() for c in contours2])
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
    cv_drawContours(cannyColor, approx, -1, (0, 255, 0), 10)
    # displayImage("contours", cannyColor)
    black = np_zeros_like(canny)
    cv_drawContours(black, approx, -1, 255, 5)
    return black


def getHeading(leftPts, rightPts, img, multiplier=1.0):
    """The multiplier is how much of the line you actually want to get back"""
    width = img.shape[1]
    height = img.shape[0]

    # average of top x value of points
    avgTopX = (leftPts[1][0] + rightPts[1][0]) // 2
    avgTopY = height * (1 - multiplier)

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

    justLine = np_zeros_like(orig)
    cv_line(justLine, p0, p1, (0, 0, 255), 3)

    overlay = cv_addWeighted(orig, 1.0, justLine, beta=0.95, gamma=0.0)
    # displayImage("overlay", overlay)

    return overlay


def displayImage(name, mat, wait=True):
    cv_imshow(name, mat)
    if wait:
        return cv_waitKey(0)
    else:
        return None


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
        img = cv_cvtColor(img, cv_COLOR_GRAY2BGR)
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
        return None

    # subsample image
    interp = cv_INTER_AREA
    smallImg = cv_resize(img, (halfW, halfH), interpolation=interp)

    # put on image
    sys_stdout.flush()
    bigImg[p0[1]:p1[1], p0[0]:p1[0]] = smallImg

    return bigImg


# example input of too close
# [[(505, 480), (147, 0)], [(518, 480), (283, 0)]]
def fixLaneData(img, whiteLines, yellowLines):
    # print(whiteLines)
    # print(yellowLines)
    width = img.shape[1]
    midPt = width // 2

    leftSide = None
    rightSide = None
    # find the right lane
    for l in whiteLines:
        if l is None:
            continue
        if l[0][0] > midPt:
            rightSide = l
        if l[0][0] < midPt:
            leftSide = l

    # estimates for average lane width
    bottomLaneWidth = 150
    topLaneWidth = 150
    yellowLane = yellowLines[0]

    if rightSide and (not yellowLane):
        # invent a yellow line
        rP0, rP1 = rightSide
        leftLane = [(rP0[0] - bottomLaneWidth, rP0[1]), (rP1[0] - topLaneWidth, rP1[1])]
        return leftLane, rightSide
    elif (not rightSide) and yellowLane:
        lP0, lP1 = yellowLane
        rightLane = [(lP0[0] + bottomLaneWidth, lP0[1]), (lP1[0] + topLaneWidth, lP1[1])]
        return yellowLane, rightLane
    elif yellowLane:
        lP0, lP1 = yellowLane
        rightLane = [(lP0[0] + bottomLaneWidth, lP0[1]), (lP1[0] + topLaneWidth, lP1[1])]
        return yellowLane, rightLane
    else:
        return None, None


def parseImage(path, hmg, invh, debug=False, lineMultiplier=1.0):
    """parseImage
    returns the line that describes the target to follow
    if debug is True, returns tuple that contains target and collage image
    """
    global printOnce
    global writer_rgb
    global SAVE_VIDEO

    # allow passing the image in directly
    if isinstance(path, str):
        img = cv_imread(path, cv_IMREAD_COLOR)
    else:
        img = path

    kSz = 5
    # displayImage('input',img)
    warped = cv_warpPerspective(img, hmg, (img.shape[1], img.shape[0]))
    warped = cv_GaussianBlur(warped, (kSz, kSz), 0)
    # kill the top of the image
    warped = cv_rectangle(warped, (0, 0), (img.shape[1], img.shape[0] // 2), (0, 0, 0), -1)
    # displayImage("warped", warped)

    # convert color space, threshold the image, remove noise
    yellowImg, whiteImg = cleanupImage(warped)
    # displayImage("smoothed", smoothed)

    # collage of images - original (w/ heading), smoothed, contours & canny, lanes
    if debug:
        collage = np_zeros_like(img)
        collage = addImageQuadrant(collage, img, 0)
        collage = addImageQuadrant(collage, cv_bitwise_or(yellowImg, whiteImg), 1)

    try:
        # apply Canny edge detection
        cannyWhite = cv_Canny(whiteImg, 100, 200)
        cannyYellow = cv_Canny(yellowImg, 100, 200)
        cannyBoth = cv_bitwise_or(cannyWhite, cannyYellow)
        # displayImage("Canny", canny)
        contourWhite = getContours(cannyWhite)
        contourYellow = getContours(cannyYellow)
        if contourWhite is None or contourYellow is None:
            print("error no contours found")
            return None
        # contourImg = cv_cvtColor(cannyColor, cv_COLOR_BGR2GRAY)

        linesWhite = houghLines(contourWhite)
        linesYellow = houghLines(contourYellow)
        if (not linesWhite) and (not linesYellow):
            print('no lines found')
            return None

        pointsWhite = getLinesPoints(cannyWhite, linesWhite, debug=debug)
        pointsYellow = getLinesPoints(cannyYellow, linesYellow, debug=debug, lineCnt=1)
        if (not pointsWhite) and (not pointsYellow):
            print('no points found')
            return None
        if debug:
            pointsWhite, laneImgWhite = pointsWhite
            leftPointsWhite, rightPointsWhite = pointsWhite
            pointsYellow, laneImgYellow = pointsYellow
        else:
            leftPointsWhite, rightPointsWhite = pointsWhite

        # print("white lanes: {}, {}".format(leftPointsWhite, rightPointsWhite))
        # print("yellow lanes: {}".format(pointsYellow))
        leftLane, rightLane = fixLaneData(cannyBoth, pointsWhite, pointsYellow)
        # print("new lanes: {}, {}".format(leftLane, rightLane))
        # sys_stdout.flush()
        if leftLane is None:
            origTarget = None
        else:
            # average middle line
            target = getHeading(leftLane, rightLane, warped, multiplier=lineMultiplier)
            ot = warpPoints(target, invh)
            # restructure
            origTarget = ((ot[0][0][0], ot[0][0][1]), (ot[1][0][0], ot[1][0][1]))
            # print(origTarget)
            if SAVE_VIDEO:
                #t = ((target[0][0][0], target[0][0][1]), (target[1][0][0], target[1][0][1]))
                writer_rgb.write(showHeading(origTarget,img))

        if not printOnce:
            printOnce = 1
            cv_imwrite("overlay.jpeg", showHeading(origTarget, img))

        #if SAVE_VIDEO:
        #    t = ((target[0][0][0], target[0][0][1]), (target[1][0][0], target[1][0][1]))
        #    writer_rgb.write(showHeading(target, warped)) 


        if debug:
            overlay = showHeading(origTarget, img)
            collage = addImageQuadrant(collage, overlay, 0)
            collage = addImageQuadrant(collage, cv_bitwise_or(contourWhite, contourYellow), 2)
            collage = addImageQuadrant(collage, cv_bitwise_or(laneImgWhite, laneImgYellow), 3)
            return origTarget, collage
        else:
            return origTarget

    except Exception as e:
        raise e
        sriter_rgb.close()
        if not debug:
            return None
        else:
            # print(e)
            return None, collage


def main():
    # imgDir = os_path.abspath("../../testimages/lowres")
    imgDir = os_path.abspath("../../testvideo/frames")
    # imgDir = os_path.abspath("../../testimages/chessboard")
    imgList = os_listdir(imgDir)
    imgs = sorted([os_path.join(imgDir, x) for x in imgList])

    hmg = getHomographyMatrix("color-lowres")
    invh = getHomographyMatrix("color-lowres", inverse=True)

    for i in imgs:
        if not "frame150" in i:
            continue
        print(os_path.basename(i))
        result = parseImage(i, hmg, invh, debug=True)
        if result is not None:
            target, overlay = result
            displayImage("overlay", overlay)


if __name__ == '__main__':
    main()
