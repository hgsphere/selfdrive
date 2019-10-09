
import sys
import os
import cv2 as cv
import numpy as np
from glob import glob
#!/usr/bin/python3

print("Obsolete! Use /camera/calibrate.py")
print("see /camera/testWarp.py for an example")
sys.exit(-1)

# dimensions we're working with
IMG_H = 480
IMG_W = 640

# top left, top right, bottom left, bottom right
imgPoints = {
    "calibre_left2_v1_1.jpeg" : [
        [279, 267], [429, 271], [183, 360], [591, 377]
    ],      # 5 x 8
    "calibre_left2_v1_2.jpeg" : [
        [260, 281], [452, 287], [117, 432], [639, 457]
    ],      # 4 x 8
    "calibre_left1_v1_1.jpeg" : [
        [247, 253], [430, 262], [93, 347], [601, 381]
    ],      # 6 x 8 chessboard inside of bigger one
    "calibre_left1_v1_2.jpeg" : [
        [264, 255], [446, 266], [106, 420], [645, 460]
    ],      # ?
    "calibre_center1.jpeg" : [
        [242, 254], [428, 258], [80, 358], [603, 373]
    ],      # 6 x 8
    "calibre_center2.jpeg" : [
        [251, 264], [448, 266], [95, 434], [645, 452]
    ],      # 4 x 8
    "calibre_centerv1_1.jpeg" : [
        [250, 257], [421, 257], [84, 367], [602, 373]
    ],      # 7 x 9
    "calibre_centerv1_2.jpeg" : [
        [252, 274], [449, 274], [96, 446], [647, 454]
    ],      # 4 x 8
    "calibre_right1_v1_1.jpeg" : [
        [249, 261], [429, 267], [22, 354], [598, 379]
    ],      # 7 x 8
    "calibre_right1_v1_2.jpeg" : [
        [213, 273], [454, 281], [-7, 426], [650, 460]
    ],      # 5 x 8
    "calibre_right2_v1_1.jpeg" : [
        [220, 255], [368, 260], [23, 349], [510, 369]
    ],      # 6 x 8
    "calibre_right2_v1_2.jpeg" : [
        [213, 265], [453, 274], [-5, 415], [645, 451]
    ],      # 5 x 8
    "straight1.jpeg" : [

    ],
    "straight2.jpeg" : [

    ],
    "curved1.jpeg" : [

    ],
    "curved2.jpeg" : [

    ],
    "curved_v1_1.jpeg" : [

    ],
    "curved_v1_2.jpeg" : [

    ],
}

src1_ref = "calibre_centerv1_1.jpeg"
src1 = np.float32(imgPoints[src1_ref])
src2_ref = "calibre_centerv1_2.jpeg"
src2 = np.float32(imgPoints[src2_ref])
dstPts = [
    [0, 0], [IMG_W, 0], [0, IMG_H], [IMG_W, IMG_H]
]
dst = np.float32(dstPts)

# transform matrices
M1Transform = cv.getPerspectiveTransform(src1, dst)
M2Transform = cv.getPerspectiveTransform(src2, dst)
M1Inverse   = cv.getPerspectiveTransform(dst, src1)
M2Inverse   = cv.getPerspectiveTransform(dst, src2)


def main():
    # images are 640 x 480
    images = glob("../../testimages/*.jpeg")

    for path in images:
        basename = os.path.basename(path)
        # print(path)
        img = cv.imread(path)

        cSrc = int(basename.split('.')[0][-1])
        if cSrc == 1:
            M = M1Transform
        elif cSrc == 2:
            M = M2Transform
        else:
            print("error!")
            sys.exit(-1)

        warped = cv.warpPerspective(img, M, (IMG_W, IMG_W))

        cv.imshow("img", img)
        cv.imshow("warped", warped)
        cv.waitKey(0)

    cv.destroyAllWindows()


if __name__ == '__main__':
    main()
