import os
import sys
import cv2 as cv
import numpy as np

sys.path.append(os.path.abspath("../utils/"))
from makeChessboard import getChessboardPoints


# dimensions we're working with
IMG_H = 480
IMG_W = 640

# image we use to calibrate - can still add more points
rgbSrcName = "calibre_centerv1_2.jpeg"
rgbSrcPts = [
    [213, 268], [258, 268], [304, 267], [349, 267], [395, 267], [442, 268], [488, 268],     # 0th row
    [204, 274], [252, 274], [301, 274], [350, 274], [400, 274], [449, 274], [500, 275],     # 1st row
    [139, 283], [191, 283], [245, 283], [298, 283], [351, 282],                             # 2nd row
            [404, 282], [458, 283], [514, 284], [570, 285],
    [119, 292], [591, 296],                                                                 # 3rd row
    [159, 305], [553, 308],                                                                 # 4th row
    [67,  319], [580, 324],                                                                 # 5th row
    [112, 339], [611, 344],                                                                 # 6th row
    [76,  363], [170, 364], [266, 364], [361, 366], [458, 367], [556, 369],                 # 7th row
    [29,  395], [240, 399], [253, 398], [365, 399], [480, 402], [596, 402],                 # 8th row
    [96,  446], [233, 450], [372, 450], [512, 452],                                         # 9th row
]
rgbChessPtsIdxs = [
    [0, 1], [0, 2], [0, 3], [0, 4], [0, 5], [0, 6], [0, 7],                     # 0th row
    [1, 1], [1, 2], [1, 3], [1, 4], [1, 5], [1, 6], [1, 7],                     # 1st row
    [2, 0], [2, 1], [2, 2], [2, 3], [2, 4], [2, 5], [2, 6], [2, 7], [2, 8],     # 2nd row
    [3, 0], [3, 8],                                                             # 3rd row
    [4, 1], [4, 7],                                                             # 4th row
    [5, 0], [5, 7],                                                             # 5th row
    [6, 1], [6, 7],                                                             # 6th row
    [7, 1], [7, 2], [7, 3], [7, 4], [7, 5], [7, 6],                             # 7th row
    [8, 1], [8, 2], [8, 3], [8, 4], [8, 5], [8, 6],                             # 8th row
    [9, 2], [8, 3], [8, 4], [8, 5],                                             # 9th row
]

depSrcName = "calibre_centerv1_1.jpeg"
depSrcPts = [
    [250, 258], [277, 258], [307, 258], [335, 258],                             # 0th row
            [365, 258], [392, 258], [422, 258], [450, 258],
    [213, 263], [243, 263], [274, 263], [304, 262], [335, 262],                 # 1st row
            [366, 262], [397, 262], [428, 262], [458, 263],
    [190, 274], [225, 274], [262, 274], [299, 274], [336, 274],                 # 3rd row
            [373, 274], [409, 274], [447, 275], [482, 274],
    [102, 317], [160, 317], [219, 319], [279, 319], [339, 319],                 # 7th row
            [398, 320], [457, 319], [518, 321], [577, 321],
    [60,  337], [128, 337], [198, 339], [269, 340], [338, 340],                 # 8th row
            [410, 340], [482, 342], [552, 342], [624, 343],
    [0,   368], [83,  368], [168, 370], [254, 371],                             # 9th row
            [340, 371], [427, 372], [515, 372], [603, 373],
]
depChessPtsIdxs = [
    [0, 1], [0, 2], [0, 3], [0, 4], [0, 5], [0, 6], [0, 7], [0, 8],             # 0th row
    [1, 0], [1, 1], [1, 2], [1, 3], [1, 4], [1, 5], [1, 6], [1, 7], [1, 8],     # 1st row
    [3, 0], [3, 1], [3, 2], [3, 3], [3, 4], [3, 5], [3, 6], [3, 7], [3, 8],     # 3rd row
    [7, 0], [7, 1], [7, 2], [7, 3], [7, 4], [7, 5], [7, 6], [7, 7], [7, 8],     # 7th row
    [8, 0], [8, 1], [8, 2], [8, 3], [8, 4], [8, 5], [8, 6], [8, 7], [8, 8],     # 8th row
    [9, 0], [9, 1], [9, 2], [9, 3], [9, 4], [9, 5], [9, 6], [9, 7],             # 9th row
]


def getHomographyMatrix(type="color"):
    if type == "color":
        rgbChessboardPts = getChessboardPoints((IMG_W, IMG_H), cols=8, rows=9, show=False)
        rgbChessPts = []

        for p in rgbChessPtsIdxs:
            rgbChessPts.append(rgbChessboardPts[p[0]][p[1]])

        hmg, status = cv.findHomography(np.array(rgbSrcPts), np.array(rgbChessPts))
        return hmg

    elif type == "depth":
        # depth
        depChessboardPts = getChessboardPoints((IMG_W, IMG_H), cols=8, rows=9, show=False)
        depChessPts = []

        for p in depChessPtsIdxs:
            depChessPts.append(depChessboardPts[p[0]][p[1]])

        hmg, status = cv.findHomography(np.array(depSrcPts), np.array(depChessPts))
        return hmg

    else:
        print("invalid option!")
        return None


def getImg(dir, name, show=True):
    srcImg = cv.imread(os.path.join(dir, name))
    if show:
        cv.imshow("src", srcImg)
        cv.waitKey(0)

    return srcImg


def main():
    pathDir = os.path.abspath("../../testimages/chessboard")
    rgbDir = os.path.join(pathDir, "rgb")
    depDir = os.path.join(pathDir, "depth")

    rgbSrc = getImg(rgbDir, rgbSrcName)
    hmg = getHomographyMatrix()
    rgbOut = cv.warpPerspective(rgbSrc, hmg, (rgbSrc.shape[1], rgbSrc.shape[0]))
    
    cv.imshow("warpedRGB", rgbOut)
    cv.waitKey(0)

    depSrc = getImg(depDir, depSrcName)
    hmgDep = getHomographyMatrix("depth")
    depOut = cv.warpPerspective(depSrc, hmgDep, (depSrc.shape[1], depSrc.shape[0]))

    cv.imshow("warpedDep", depOut)
    cv.waitKey(0)

    cv.destroyAllWindows()


if __name__ == '__main__':
    main()
