from cv2 import moments
from numpy import array as np_array
from math import sqrt

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
        M = moments(np_array(p))
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
    Also returns the straight-line distance to that feature.
    """
    if l is None:
        l = allFeatures

    wMax = IMG_W
    hMax = IMG_H
    # distance formula
    distMax = sqrt(pow(wMax - x, 2) + pow(hMax - y, 2))
    retFeature = None

    # find minimum distance
    for f in l:
        xn, yn = f.center
        dist = sqrt(pow(xn - x, 2) + pow(yn - y, 2))
        if dist < distMax:
            distMax = dist
            retFeature = f

    return retFeature, distMax


def findClosestStopLine(x, y):
    return findClosestFeature(x, y, stopLines)


def findClosestCrosswalk(x, y):
    return findClosestFeature(x, y, crosswalks)


def findClosestIntersection(x, y):
    return findClosestFeature(x, y, intersections)


# for the purposes of the graph walker, define some stop line coordinates manually
#  that correspond to points already on the graph
graphStopLines = {
    "303,29"    : "stopLine0",      # actual (303, 30)
    "237,161"   : "stopLine1",      # actual (238, 161)
    "833,1400"  : "stopLine2",      # actual (820, 1400)
    "754,1538"  : "stopLine3",      # actual (754, 1539)
    "341,672"   : "stopLine4",      # actual (345, 672)
    "394,874"   : "stopLine5",      # actual (395, 870)
    "590,828"   : "stopLine6",      # actual (590, 824)
    "546,620"   : "stopLine7",      # actual (544, 622)
}

# points are labeled based on the next stop line in the directed graph
# in other words, if you keep following from where the point is, the next
# stop line to be hit is the one associated with the point
graphCrossoverPts = {
    "609,650"   : "stopLine0",
    "347,579"   : "stopLine1",
    "587,919"   : "stopLine2",
    "306,871"   : "stopLine3",
    "171,216"   : "stopLine4",
    "704,1475"  : "stopLine5",
    "894,1338"  : "stopLine6",
    "379,97"    : "stopLine7",
}

# points are labeled based on the stop line they immediately follow
graphTurningPts = {
    "282,29"    : "stopLine0",
    "237,127"   : "stopLine1",
    "833,1444"  : "stopLine2",
    "805,1539"  : "stopLine3",
}

# left, center, right
graphTurningPtsComplex = {
    "stopLine4" : ["397,711", "390,716", "400,752"],
    "stopLine5" : ["431,836", "447,821", "486,829"],
    "stopLine6" : ["534,787", "539,780", "539,736"],
    "stopLine7" : ["496,673", "474,690", "478,672"],
}

# labeled same as crossover points
graphStraightPts = {
    "583,676"   : "stopLine0",
    "363,597"   : "stopLine1",
    "569,901"   : "stopLine2",
    "360,818"   : "stopLine3",
    "172,165"   : "stopLine4",
    "730,1475"  : "stopLine5",
    "893,1416"  : "stopLine6",
    "350,97"    : "stopLine7",
}


def getStopLineCoordinates(name):
    """Gets the coordinates of a stop line based on its name."""
    for p, n in graphStopLines.items():
        if n == name:
            return p
    
    return None

def getCrossoverPt(name):
    for p, n in graphCrossoverPts.items():
        if n == name:
            return p

    return None

def getTurningPt(name, direction):
    if int(name[-1]) < 4:
        return getTurningPtSimple(name)
    else:
        return getTurningPtFanout(name, direction)

def getTurningPtSimple(name):
    for p, n in graphTurningPts.items():
        if n == name:
            return p
    return None

# arranged left, straight, right
def getTurningPtFanout(name, direction):
    for n, pts in graphTurningPtsComplex.items():
        if n == name:
            if direction == 6:
                return pts[0]
            elif direction == 4:
                return pts[1]
            elif direction == 5:
                return pts[2]
            else:
                print("Error, next turn variable is incorrect!")
                return pts[1]

    print("Cannot find point named {}".format(name))
    return None

def getStraightPt(name):
    for p, n in graphStraightPts.items():
        if n == name:
            return p

    return None
