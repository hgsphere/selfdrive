import os
import sys
import cv2 as cv
import numpy as np
from glob import glob

sys.path.append(os.path.abspath(os.getcwd()))
from calibrate import getHomographyMatrix


def main():
    rgbImages = glob("../../testimages/lanes/rgb/*.jpeg")
    depImages = glob("../../testimages/lanes/depth/*.jpeg")

    hmgRgb = getHomographyMatrix("color")
    hmgDep = getHomographyMatrix("depth")

    for path in rgbImages:
        # basename = os.path.basename(path)
        img = cv.imread(path)

        warped = cv.warpPerspective(img, hmgRgb, (img.shape[1], img.shape[0]))

        cv.imshow("src", img)
        cv.imshow("warped", warped)
        cv.waitKey(0)

    for path in depImages:
        # basename = os.path.basename(path)
        img = cv.imread(path)

        warped = cv.warpPerspective(img, hmgDep, (img.shape[1], img.shape[0]))

        cv.imshow("src", img)
        cv.imshow("warped", warped)
        cv.waitKey(0)

    cv.destroyAllWindows()


if __name__ == '__main__':
    main()
