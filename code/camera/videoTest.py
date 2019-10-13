import os
import sys
import cv2 as cv

sys.path.append(os.path.abspath(os.getcwd()))
from calibrate import getHomographyMatrix
from findLines import parseImage, displayImage


def main():

    frameRate = 30
    resolution = (640, 480)

    # change this directory and name to change which video is processed
    vidDir = "../../testvideo/clips"
    vidName = "clip0.avi"
    vidPath = os.path.join(vidDir, vidName)
    vid = cv.VideoCapture(vidPath)

    inName = vidName.split(".")
    outName = inName[0] + "-debug.avi"
    outPath = os.path.join(vidDir, outName)
    out = cv.VideoWriter(outPath, cv.VideoWriter_fourcc('M','J','P','G'), frameRate, resolution)

    hmg = getHomographyMatrix("color")
    invh = getHomographyMatrix("color", inverse=True)

    # for i in range(60):
    while True:
        ret, frame = vid.read()

        if ret:
            overlay = parseImage(frame, hmg, invh, debug=True)
            out.write(overlay)

            # debugging stuff
            # cv.imshow("frame", frame)
            # keyVal = displayImage("overlay", overlay)
            # if chr(keyVal & 255) == 's':
            #     print("saving image")
            #     cv.imwrite("../../testimages/frames/rgb/frame{}.jpeg".format(i), frame)

        else:
            break

    vid.release()
    out.release()


if __name__ == '__main__':
    main()
