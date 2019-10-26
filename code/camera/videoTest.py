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
        overlay = frame
    else:
        overlay = drawCrossBox(frame, crossbox)

    if crossLines:
        # if debug:
        #     print("Found a line")
        overlay = drawCrossLines(frame, crossLines)

    out.write(overlay)

    if debug:
        keyVal = displayImage("frame", overlay)
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

    frameRate = 60
    resolution = (424, 240)

    # change this directory and name to change which video is processed
    if test == "lanes":
        vidDir = "../../testvideo"
        vidName = "output-rgb-lowres.avi"
    elif test == "stopLines":
        # vidDir = "../../testvideo/stopLines"
        # vidName = "stopLine6.avi"
        vidDir = "../../testvideo"
        vidName = "output-rgb-lowres.avi"
    elif test == "all":
        vidDir = "../../testvideo"
        vidName = "output-rgb-lowres.avi"
    else:
        return -1

    vidPath = os.path.join(vidDir, vidName)
    vid = cv.VideoCapture(vidPath)

    inName = vidName.split(".")
    outName = inName[0] + "-debug3.avi"
    outPath = os.path.join(vidDir, outName)
    out = cv.VideoWriter(outPath, cv.VideoWriter_fourcc('M','J','P','G'), frameRate, resolution)

    hmg = getHomographyMatrix("color-lowres")
    invh = getHomographyMatrix("color-lowres", inverse=True)

    dbgSecond = 17
    dbgPeriod = 1
    i = 0
    dbg = False
    # for i in range(60):
    while True:
        ret, frame = vid.read()

        if ret:
            if test == "lanes":
                testLaneDetect(frame, hmg, invh, out, vidDir, i, debug=dbg)
            elif test == "stopLines":
                testCrosswalkDetect(frame, hmg, invh, out, vidDir, i, debug=dbg)
            elif test == "all":
                testAll(frame, hmg, invh, out, vidDir, i)
            else:
                break
            i += 1
            # if i == (dbgSecond*frameRate):
            #     dbg = True
            # elif i == ((dbgSecond+dbgPeriod)*frameRate):
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
