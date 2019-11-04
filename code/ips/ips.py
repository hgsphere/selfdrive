#!/usr/bin/python3
import json
import os
import sys
from requests import get as rget
import cv2 as cv
import numpy as np
from math import sqrt
from multiprocessing import Value
from networkx import (DiGraph as nx_DiGraph,
                      from_dict_of_dicts as nx_from_dict_of_dicts,
                      shortest_path as nx_shortest_path)

sys.path.append(os.getcwd())
from digraph import (drawLine, drawPt, getPtName, decodePtName)
import features

# Import global to reference files by full path
sys.path.append(os.path.abspath("../systemStructure"))
import globals

##############################################################################
# The x,y coordinate system for the Indoor Positioning System (IPS)
# is defined to have the origin in the upper right corner of the image.
# The x-axis goes down the right side of the image, and the y-axis along
# the top.
# But, we rotated the image so it matches the OpenCV way of doing things.
##############################################################################


validColors = ["Green", "Red", "Purple", "Light Blue", "Yellow"]


def displayRouteImg(name, img, wait=True, targetHeight=740):
    """Resize image to fit the screen."""
    width = img.shape[1]
    height = img.shape[0]
    r = targetHeight / float(height)
    targetWidth = int(width * r)

    sz = (targetWidth, targetHeight)
    smaller = cv.resize(img, sz, interpolation=cv.INTER_AREA)

    cv.imshow(name, smaller)
    if wait:
        while True:
            key = cv.waitKey(0) & 0xFF
            if key == ord('q'):
                break
    else:
        return


def computeTurnDirection(nodes):
    """Compute which direction to turn.  Accepts a slice of the path, only 3 nodes needed."""
    if len(nodes) != 5:
        print("Takes exactly 3 nodes")
        raise IndexError
    n0 = nodes[0]
    n1 = nodes[1]
    n2 = nodes[4]
    # n0, n1, n2 = nodes
    angleTolerance = 5.0

    side_a = sqrt(pow(n0[0] - n1[0], 2) + pow(n0[1] - n1[1], 2))
    side_b = sqrt(pow(n0[0] - n2[0], 2) + pow(n0[1] - n2[1], 2))
    side_c = sqrt(pow(n1[0] - n2[0], 2) + pow(n1[1] - n2[1], 2))

    # print(dist_plan,dist_close,dist_future)
    # calculate the angle between the car and perdicted GPS path
    GPS_angle = (180 / np.pi) * np.arccos(
        (side_a ** 2 + side_b ** 2 - side_c ** 2) / (2 * abs(side_a * side_b)))

    # slopes
    # if (n1[0] - n0[0]) == 0:
    #     s0 = 1000
    # else:
    #     s0 = (n1[1] - n0[1]) / (n1[0] - n0[0])
    #
    # if (n2[0] - n1[0]) == 0:
    #     s1 = 1000
    # else:
    #     s1 = (n2[1] - n1[1]) / (n2[0] - n1[0])

    # sdiff = abs(s1) - abs(s0)
    print("We are going to turn, based on slope difference {}".format(GPS_angle))
    print(nodes)

    # make decision
    if abs(GPS_angle) < angleTolerance:
        return "Force_Forward"
    elif GPS_angle > 0:
        return "Force_Right_Turn"
    elif GPS_angle < 0:
        return "Force_Left_Turn"
    else:
        print("ERROR!!!!!")
        return "Lane_Follow"


# globals for coordinate values
longitude = Value("d", 0.0)
latitude = Value("d", 0.0)

Glat = 0
Glon = 0

def getCurrentCoor(color="Yellow"):
    global Glat
    global Glon
    """Get the current IPS location of the car."""
    # validate request
    if color not in validColors:
        return 0, 0

    # request data
    URL = "http://192.168.1.8:8080/{}".format(color)
    r = rget(url=URL)

    # extract data
    try:
        coorString = r.text
        coordinates = coorString.split()
        if coordinates is not None:
            lat = float(coordinates[0])
            lon = float(coordinates[1])
            Glat = lat
            Glon = lon
        else:
            lat = Glat
            lon = Glon
    except Exception as e:
        lat = -1
        lon = -1

    return lat, lon

#def pollCoordinates(ips_routeManagerQ):
def pollCoordinates(lat, lon):
    #global latitude, longitude

    while True:
        _lat, _lon = getCurrentCoor()
        lat.value = _lat
        lon.value = _lon
        #latitude = lat
        #longitude = lon
        # ips_routeManagerQ.put_nowait(coords)


class IPS(object):
    """Implements a path finding algorithm for navigating the course."""

    def __init__(self):
        # we're using a directed graph
        self.graph = nx_DiGraph()
        # load in the data from the graph file
        with open(os.path.join(globals.code_base_dir,"ips/graph.json"), 'r') as jf:
            self.graphDict = json.load(jf)
            self.graph = nx_from_dict_of_dicts(self.graphDict, create_using=self.graph)
        # also keep the image around
        image_path = os.path.join(globals.code_base_dir,"ips/Global.jpg")
        self.image = cv.imread(image_path)
        # determine the average distance between each node
        distances = []
        for node, edges in self.graphDict.items():
            xn, yn = decodePtName(node)
            x, y = xn, yn
            # we'll just do the first possible edge
            for nm, attr in edges.items():
                x, y = decodePtName(nm)
                break
            dist = sqrt(pow(xn - x, 2) + pow(yn - y, 2))
            if dist > 0:
                distances.append(dist)
        self.avg_dst = int(np.mean(distances))

    def findNextStopLine(self, x, y):
        """Instead of finding straight line distance,
        walk along the graph until you run into a feature.
        This will return a path just like the shortest path finder, with the name.
        """
        # get the closest graph point first
        x_g, y_g = self.findClosestGraphPoint(x, y)
        nextNode = getPtName(x_g, y_g)

        # now walk until feature is found
        while True:
            # check if we've hit a feature
            if nextNode in list(features.graphStopLines.keys()):
                name = features.graphStopLines[nextNode]
                break
            else:
                # we can always assume it will be the first element
                #  because will run into feature before fan out
                nextNode = next(iter(self.graph.neighbors(nextNode)))

        # now call the pathfinder
        return self.findPath(x, y, *decodePtName(nextNode)), name

    def findClosestGraphPoint(self, x, y, getDist=False):
        """Given any point on the image, what is the closest point in the graph?"""
        wMax = self.image.shape[1]
        hMax = self.image.shape[0]
        # distance formula
        distMax = sqrt(pow(wMax - x, 2) + pow(hMax - y, 2))

        # find minimum distance
        for node in self.graph:
            xn, yn = decodePtName(node)
            dist = sqrt(pow(xn - x, 2) + pow(yn - y, 2))
            if dist < distMax:
                wMax = xn
                hMax = yn
                distMax = dist

        # print("Closest point to {}, {} is {}, {}".format(x, y, wMax, hMax))
        if getDist:
            return wMax, hMax, distMax
        else:
            return wMax, hMax

    def findPath(self, x0, y0, x1, y1):
        """Find the shortest point between two points on the graph.
        Returns a list of x,y tuples.  May or may not include the original points.
        """
        # find the points in the graph that is closest to what we are given
        c0 = self.findClosestGraphPoint(x0, y0)
        c1 = self.findClosestGraphPoint(x1, y1)
        # the name of the node in the graph
        n0 = getPtName(*c0)
        n1 = getPtName(*c1)

        path = nx_shortest_path(self.graph, source=n0, target=n1)
        # convert to int
        pathInt = [decodePtName(n) for n in path]
        return pathInt

    def displayPath(self, path):
        """path is a list of node names in the graph.
        This is great because we have encoded the coordinates in the node names.
        """
        img = np.copy(self.image)
        for i in range(len(path)-1):
            drawLine(img, path[i], path[i+1])

        drawPt(img, *path[-1])
        return img

    def displayDirectedGraph(self):
        """Display the entire graph overlaid on the image."""
        img = np.copy(self.image)
        for node, edges in self.graphDict.items():
            x, y = decodePtName(node)
            drawPt(img, x, y)
            for nm, attr in edges.items():
                pt1 = decodePtName(nm)
                drawLine(img, (x, y), pt1)
        return img


def pointClick(event, x, y, flags, params):
    """Callback function for showcasing the feature detection."""

    if event == cv.EVENT_LBUTTONUP:
        ips = params[0]

        path, name = ips.findNextStopLine(x, y)
        print("{} at {}".format(name, path[-1]))
        img = ips.displayPath(path)

        # visualize
        RED = (0, 0, 255)
        # where you clicked
        drawPt(img, x, y, color=RED)
        # nearest graph point
        drawLine(img, (x, y), path[0], color=RED)

        cv.imshow("features", img)


def testFeatureFinding(ips):
    """Displays the map of the course.  If you click on any pixel, it will draw
    a circle on that pixel, and a circle at the center of the nearest feature.
    Press 'q' to quit.
    """

    cv.namedWindow("features")
    cv.setMouseCallback("features", pointClick, param=(ips,))
    cv.imshow("features", ips.image)

    while True:
        key = cv.waitKey(0) & 0xFF

        if key == ord('q'):
            break


def testPathFinding(ips):
    """Finds the shortest path between p0 and p1.
    These can be changed to test the correctness of paths.
    """

    cv.namedWindow("path")
    p0 = 549, 613
    p1 = 770, 1064
    RED = (0, 0, 255)

    path = ips.findPath(*p0, *p1)
    pathImg = ips.displayPath(path)

    drawPt(pathImg, *p0, color=RED)
    drawLine(pathImg, p0, path[0], color=RED)
    drawPt(pathImg, *p1, color=RED)
    drawLine(pathImg, path[-1], p1, color=RED)

    displayRouteImg("path", pathImg)
    return


def main():
    ips = IPS()
    # cv.imwrite("graphMap.jpeg", ips.displayDirectedGraph())
    displayRouteImg("path", ips.displayDirectedGraph())

    testPathFinding(ips)
    # testFeatureFinding(ips)

    cv.destroyAllWindows()


if __name__ == '__main__':
    main()
