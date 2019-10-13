from cv2 import minAreaRect, approxPolyDP, boxPoints
from numpy import int0

class contourPlus(object):
    def __init__(self, ar):
        """This has the contour array, but also more data."""
        self.array = ar
        self.rect = minAreaRect(ar)
        # minAreaRect returns tuple of ((centerX, centerY), (width, height), angle)
        self.approx = approxPolyDP(ar, epsilon=0.1, closed=True)

    def getSize(self):
        # (width, height)
        return self.rect[1]

    def getArea(self):
        return self.rect[1][0] * self.rect[1][1]

    def getProportion(self):
        return max((self.rect[1][0] / self.rect[1][1]), (self.rect[1][1] / self.rect[1][0]))

    def getBoxAsContour(self):
        # drawable with drawContours
        return [int0(boxPoints(self.rect))]
