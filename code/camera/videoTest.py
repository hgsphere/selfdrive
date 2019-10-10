import os
import sys
import cv2 as cv

sys.path.append(os.path.abspath(os.getcwd()))
from calibrate import getHomographyMatrix
from findLines import parseImage, displayImage


def main():

    vid = cv.VideoCapture("../../testvideo/output-rgb.avi")

    out = cv.VideoWriter("../../testvideo/linefollow-output-rgb-debug.avi",
                         cv.VideoWriter_fourcc('M','J','P','G'), 30, (640, 480))

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
