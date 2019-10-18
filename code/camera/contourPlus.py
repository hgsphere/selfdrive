from numpy import (int0,
                   array,
                   float32)
from cv2 import (minAreaRect,
                 approxPolyDP,
                 boxPoints,
                 perspectiveTransform)


class contourPlus(object):
    def __init__(self, ar):
        """This has the contour array, but also more data."""
        self.array = ar
        try:
            self.rect = minAreaRect(ar)
        except Exception as e:
            print(ar)
            raise e
        # minAreaRect returns tuple of ((centerX, centerY), (width, height), angle)
        self.approx = approxPolyDP(ar, epsilon=0.1, closed=True)
        # unpack
        self.width = min(self.rect[1][1], self.rect[1][0])
        self.height = max(self.rect[1][1], self.rect[1][0])
        self.center = self.rect[0]
        self.angle = self.rect[2]

    def __str__(self):
        return str(self.rect)

    def getSize(self):
        # (width, height)
        return self.rect[1]

    def getArea(self):
        return self.height * self.width

    def getProportion(self):
        return self.height / self.width
        # return max((self.rect[1][0] / self.rect[1][1]), (self.rect[1][1] / self.rect[1][0]))

    def getBoxAsContour(self):
        # drawable with drawContours
        return int0(boxPoints(self.rect))


"""input needs to look like:
[[457.97925 459.21854]
 [141.96892 454.98624]
 [143.08673 371.52353]
 [459.09705 375.75583]]
"""
def warpPoints(pts, hmg):
    # reconstruct points
    newPoints = []
    for p in pts:
        x, y = p
        newPoints.append(array([[x, y]], dtype=float32))
    newPoints = array(newPoints)

    # warp back to perspective
    warpedPts = perspectiveTransform(newPoints, hmg)

    # change back to correct format
    origPts = int0([[p[0]] for p in warpedPts])

    """return looks like
    [[[363 467]]

     [[659 323]]]
    """
    return origPts

