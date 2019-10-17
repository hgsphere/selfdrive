import os
import sys
import cv2 as cv
from datetime import datetime

sys.path.append(os.path.abspath(os.getcwd()))
from calibrate import getHomographyMatrix
from findLines import parseImage, displayImage, showHeading
from findStopLine import findStopLine, drawCrossBox, drawCrossLines


def testLaneDetect(frame, hmg, invh, out, vidDir, i, debug=False):
    target = parseImage(frame, hmg, invh, debug=debug)
    if debug:
        target, overlay = target
    else:
        overlay = showHeading(target, frame)
    out.write(overlay)

    # debugging stuff
    if debug:
        cv.imshow("frame", frame)
        keyVal = displayImage("overlay", overlay)
        if chr(keyVal & 255) == 's':
            pName = os.path.join(vidDir, "frames/frame{}.jpeg".format(i))
            print("saving {}".format(pName))
            cv.imwrite(pName, frame)


def testCrosswalkDetect(frame, hmg, invh, out, vidDir, i, debug=False):
    crossbox, crossLines = findStopLine(frame, hmg, invh)

    if crossbox is None:
        pass
    else:
        drawCrossBox(frame, crossbox)

    if crossLines:
        drawCrossLines(frame, crossLines)

    out.write(frame)

    if debug:
        keyVal = displayImage("frame", frame)
        if chr(keyVal & 255) == 's':
            pName = os.path.join(vidDir, "frames/frame{}.jpeg".format(i))
            print("saving {}".format(pName))
            cv.imwrite(pName, frame)


def testAll(frame, hmg, invh, out, vidDir, i):
    target = parseImage(frame, hmg, invh)
    overlay = showHeading(target, frame)
    crossbox, crossLines = findStopLine(frame, hmg, invh)

    if crossbox is not None:
        overlay = drawCrossBox(overlay, crossbox)
    if crossLines:
        overlay = drawCrossLines(overlay, crossLines)

    out.write(overlay)


def main():
    # test = "lanes"
    # test = "stopLines"
    test = "all"

    frameRate = 30
    resolution = (640, 480)

    # change this directory and name to change which video is processed
    if test == "lanes":
        vidDir = "../../testvideo"
        vidName = "output-rgb.avi"
    elif test == "stopLines":
        vidDir = "../../testvideo/stopLines"
        vidName = "stopLine6.avi"
    elif test == "all":
        vidDir = "../../testvideo"
        vidName = "output-rgb.avi"
    else:
        return -1

    vidPath = os.path.join(vidDir, vidName)
    vid = cv.VideoCapture(vidPath)

    inName = vidName.split(".")
    outName = inName[0] + "-debug2.avi"
    outPath = os.path.join(vidDir, outName)
    out = cv.VideoWriter(outPath, cv.VideoWriter_fourcc('M','J','P','G'), frameRate, resolution)

    hmg = getHomographyMatrix("color")
    invh = getHomographyMatrix("color", inverse=True)

    i = 0
    dbg = False
    # for i in range(60):
    while True:
        ret, frame = vid.read()

        if ret:
            if test == "lanes":
                testLaneDetect(frame, hmg, invh, out, vidDir, i, debug=dbg)
            elif test == "stopLines":
                testCrosswalkDetect(frame, hmg, invh, out, vidDir, i, debug=False)
            elif test == "all":
                testAll(frame, hmg, invh, out, vidDir, i)
            else:
                break
            i += 1
            # if i == (7*frameRate):
            #     dbg = True
            # elif i == (8*frameRate):
            #     dbg = False
            #     cv.destroyAllWindows()

        else:
            break

    vid.release()
    out.release()
    print("Wrote to {}".format(outPath))


if __name__ == '__main__':
    begin = datetime.now()
    main()
    end = datetime.now()
    print("Took {}".format(end - begin))
