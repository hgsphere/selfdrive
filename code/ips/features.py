import cv2 as cv
import numpy as np

##############################################################################
# In this file, we define features of the map in terms of their pixel
# coordinates.
##############################################################################


class Feature(object):
    def __init__(self, n, p):
        self.name = n
        self.points = p


class FeatureLine(Feature):
    def __init__(self, n, p):
        super().__init__(n, p)
        p0, p1 = p[0], p[1]
        self.center = int((p0[0]+p1[0])/2), int((p0[1]+p1[1])/2)


class FeaturePolygon(Feature):
    def __init__(self, n, p):
        super().__init__(n, p)
        M = cv.moments(np.array(p))
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        self.center = cX, cY


# image dimensions
IMG_W = 1024
IMG_H = 1600


# image features

stopSign0  = FeaturePolygon("stopSign0",
                [ (148,    5), (141,  126), (266,  124), (267,    3) ])
stopLine0  = FeatureLine("stopLine0",
                [ (303,    3), (303,   58) ])
stopLine1  = FeatureLine("stopLine1",
                [ (207,  161), (270,  161) ])
crosswalk0 = FeaturePolygon("crosswalk0",
                [ (268,   10), (266,  125), (291,  124), (292,    9) ])
crosswalk1 = FeaturePolygon("crosswalk1",
                [ (151,  126), (151,  151), (267,  151), (266,  125) ])

stopSign1  = FeaturePolygon("stopSign1",
                [ (797, 1441), (796, 1566), (919, 1567), (925, 1440) ])
stopLine2  = FeatureLine("stopLine2",
                [ (791, 1401), (850, 1400) ])
stopLine3  = FeatureLine("stopLine3",
                [ (754, 1512), (754, 1567) ])
crosswalk2 = FeaturePolygon("crosswalk2",
                [ (795, 1417), (796, 1441), (912, 1439), (912, 1415) ])
crosswalk3 = FeaturePolygon("crosswalk3",
                [ (771, 1442), (769, 1558), (796, 1558), (796, 1441) ])

fourWayStop= FeaturePolygon("fourWayStop",
                [ (447,  636), (356,  727), (360,  770), (452,  860),
                  (490,  862), (582,  767), (582,  727), (489,  635) ])
stopLine4  = FeatureLine("stopLine4",
                [ (366,  650), (324,  694) ])
stopLine5  = FeatureLine("stopLine5",
                [ (373,  847), (418,  894) ])
stopLine6  = FeatureLine("stopLine6",
                [ (567,  848), (613,  801) ])
stopLine7  = FeatureLine("stopLine7",
                [ (524,  601), (565,  643) ])
crosswalk4 = FeaturePolygon("crosswalk4",
                [ (421,  613), (334,  704), (356,  727), (447,  636) ])
crosswalk5 = FeaturePolygon("crosswalk5",
                [ (360,  770), (336,  795), (426,  885), (452,  860) ])
crosswalk6 = FeaturePolygon("crosswalk6",
                [ (582,  767), (490,  862), (516,  888), (605,  792) ])
crosswalk7 = FeaturePolygon("crosswalk7",
                [ (489,  635), (582,  727), (604,  702), (514,  610) ])


# group the different kinds of features together
stopLines  = [stopLine0, stopLine1, stopLine2, stopLine3,
              stopLine4, stopLine5, stopLine6, stopLine7]
crosswalks = [crosswalk0, crosswalk1, crosswalk2, crosswalk3,
              crosswalk4, crosswalk5, crosswalk6, crosswalk7]
intersections = [stopSign0, stopSign1, fourWayStop]

# now everything all in one list
allFeatures = []
allFeatures.extend(stopLines)
allFeatures.extend(crosswalks)
allFeatures.extend(intersections)


def findClosestFeature(x, y, l=None):
    """Given a point on the image, what is the closest feature on the road?

    This is in raw distance, not necessarily along a valid path.
    Distance is computed to the center-point of the feature.
    Returns the Feature object it's closest to, or None.
    """
    if l is None:
        l = allFeatures

    wMax = IMG_W
    hMax = IMG_H
    # distance formula
    distMax = pow(wMax - x, 2) + pow(hMax - y, 2)
    retFeature = None

    # find minimum distance
    for f in l:
        xn, yn = f.center
        dist = pow(xn - x, 2) + pow(yn - y, 2)
        if dist < distMax:
            distMax = dist
            retFeature = f

    return retFeature


def findClosestStopLine(x, y):
    return findClosestFeature(x, y, stopLines)


def findClosestCrosswalk(x, y):
    return findClosestFeature(x, y, crosswalks)


def findClosestIntersection(x, y):
    return findClosestFeature(x, y, intersections)
