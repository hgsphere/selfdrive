from os import (path as os_path,
                getcwd as os_getcwd)
from sys import (path as sys_path)
from cv2 import (imread as cv_imread,
                imshow as cv_imshow,
                waitKey as cv_waitKey,
                destroyAllWindows as cv_destroyAllWindows,
                warpPerspective as cv_warpPerspective)
from glob import glob

sys_path.append(os_path.abspath(os_getcwd()))
from calibrate import getHomographyMatrix


def main():
    rgbImages = glob("../../testimages/lanes/rgb/*.jpeg")
    depImages = glob("../../testimages/lanes/depth/*.jpeg")

    hmgRgb = getHomographyMatrix("color")
    hmgDep = getHomographyMatrix("depth")

    for path in rgbImages:
        # basename = os_path.basename(path)
        img = cv_imread(path)

        warped = cv_warpPerspective(img, hmgRgb, (img.shape[1], img.shape[0]))

        cv_imshow("src", img)
        cv_imshow("warped", warped)
        cv_waitKey(0)

    for path in depImages:
        # basename = os_path.basename(path)
        img = cv_imread(path)

        warped = cv_warpPerspective(img, hmgDep, (img.shape[1], img.shape[0]))

        cv_imshow("src", img)
        cv_imshow("warped", warped)
        cv_waitKey(0)

    cv_destroyAllWindows()


if __name__ == '__main__':
    main()
