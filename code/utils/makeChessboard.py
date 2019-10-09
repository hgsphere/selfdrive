import cv2 as cv
import numpy as np
import pprint

# help from
# https://answers.opencv.org/question/27917/how-to-create-a-chess-board/


def blackWhiteSwap(color):
    if color == 0:
        return 255
    else:
        return 0


def getChessboardPoints(shape=(640, 480), cols=8, rows=9, show=False):
    # dimensions we're working with
    IMG_W = shape[0]
    IMG_H = shape[1]

    # step size
    stepW = IMG_W // cols
    stepH = IMG_H // rows

    pts = []

    chessboard = np.zeros([IMG_H, IMG_W], dtype=np.uint8)
    color = 0

    for j in range(0, IMG_H, stepH):
        color = blackWhiteSwap(color)
        rowPts = []

        for i in range(0, IMG_W, stepW):
            rowPts.append([i, j])
            cv.rectangle(chessboard, (i, j), (i+stepW, j+stepH), color, thickness=cv.FILLED)
            color = blackWhiteSwap(color)
        rowPts.append([i, j+stepW])
        pts.append(rowPts)

    if show:
        cv.imshow("chessboard", chessboard)
        cv.waitKey(0)

    return pts


def main():
    pts = getChessboardPoints(show=True)

    pp = pprint.PrettyPrinter(indent=2)
    pp.pprint(pts)
    print(len(pts))
    print(len(pts[0]))


if __name__ == '__main__':
    main()
