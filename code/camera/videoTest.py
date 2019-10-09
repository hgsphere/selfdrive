import os
import sys
import cv2 as cv

sys.path.append(os.path.abspath(os.getcwd()))
from calibrate import getHomographyMatrix
from findLines import parseImage


def main():

    vid = cv.VideoCapture("../../testvideo/output-rgb.avi")

    out = cv.VideoWriter("../../testvideo/linefollow-output-rgb.avi",
                         cv.VideoWriter_fourcc('M','J','P','G'), 30, (640, 480))

    hmg = getHomographyMatrix("color")
    invh = getHomographyMatrix("color", inverse=True)

    # for _ in range(60):
    while True:
        ret, frame = vid.read()

        if ret:
            overlay = parseImage(frame, hmg, invh)
            out.write(overlay)

        else:
            break

    vid.release()
    out.release()


if __name__ == '__main__':
    main()
