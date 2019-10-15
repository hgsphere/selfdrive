import os
import sys
import cv2 as cv

sys.path.append(os.path.abspath(os.getcwd()))
from calibrate import getHomographyMatrix
from findLines import parseImage, displayImage
from findStopLine import findStopLine


def testLaneDetect(frame, hmg, invh, out, vidDir, i, debug=True):
    overlay = parseImage(frame, hmg, invh, debug=debug)
    out.write(overlay)
    # debugging stuff
    cv.imshow("frame", frame)
    keyVal = displayImage("overlay", overlay)
    if chr(keyVal & 255) == 's':
        print("saving image")
        cv.imwrite(os.path.join(vidDir, "frames/frame{}.jpeg".format(i)), frame)


def testCrosswalkDetect(frame, hmg, invh, out):
    crossbox = findStopLine(frame, hmg, invh)

    if crossbox is None:
        pass
    else:
        pts = [(p[0][0], p[0][1]) for p in crossbox]
        cv.line(frame, pts[0], pts[1], (0, 255, 0), 5)
        cv.line(frame, pts[1], pts[2], (0, 255, 0), 5)
        cv.line(frame, pts[2], pts[3], (0, 255, 0), 5)
        cv.line(frame, pts[3], pts[0], (0, 255, 0), 5)

    out.write(frame)


def main():
    # test = "lanes"
    test = "stopLines"

    frameRate = 30
    resolution = (640, 480)

    # change this directory and name to change which video is processed
    if test == "lanes":
        vidDir = "../../testvideo"
        vidName = "linefollow-output-rgb.avi"
    elif test == "stopLines":
        vidDir = "../../testvideo/stopLines"
        vidName = "stopLine7.avi"
    else:
        return -1

    vidPath = os.path.join(vidDir, vidName)
    vid = cv.VideoCapture(vidPath)

    inName = vidName.split(".")
    outName = inName[0] + "-debug.avi"
    outPath = os.path.join(vidDir, outName)
    out = cv.VideoWriter(outPath, cv.VideoWriter_fourcc('M','J','P','G'), frameRate, resolution)

    hmg = getHomographyMatrix("color")
    invh = getHomographyMatrix("color", inverse=True)

    for i in range(60):
    # while True:
        ret, frame = vid.read()

        if ret:
            if test == "lanes":
                testLaneDetect(frame, hmg, invh, out, vidDir, i, debug=True)
            elif test == "stopLines":
                testCrosswalkDetect(frame, hmg, invh, out)
            else:
                break

        else:
            break

    vid.release()
    out.release()


if __name__ == '__main__':
    main()
