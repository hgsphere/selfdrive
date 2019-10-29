#!/usr/bin/python3
import json
import os
import sys
from requests import get as rget
import cv2 as cv
import numpy as np
from networkx import (DiGraph as nx_DiGraph,
                      from_dict_of_dicts as nx_from_dict_of_dicts,
                      shortest_path as nx_shortest_path)

sys.path.append(os.getcwd())
from digraph import (drawLine, drawPt, getPtName, decodePtName)
import features


##############################################################################
# The x,y coordinate system for the Indoor Positioning System (IPS)
# is defined to have the origin in the upper right corner of the image.
# The x-axis goes down the right side of the image, and the y-axis along
# the top.
# But, we rotated the image so it matches the OpenCV way of doing things.
##############################################################################


validColors = ["Green", "Red", "Purple", "Light Blue", "Yellow"]


def displayRouteImg(name, img, wait=True):
    """Resize image to fit the screen."""
    width = img.shape[1]
    height = img.shape[0]
    targetHeight = 960
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


class IPS(object):
    """Implements an interface with the IPS, as well as path finding algorithms."""

    def __init__(self):
        # we're using a directed graph
        self.graph = nx_DiGraph()
        # load in the data from the graph file
        with open("./graph.json", 'r') as jf:
            self.graphDict = json.load(jf)
            self.graph = nx_from_dict_of_dicts(self.graphDict, create_using=self.graph)
        # also keep the image around
        image_path = "./Global.jpg"
        self.image = cv.imread(image_path)

    def findClosestGraphPoint(self, x, y):
        """Given any point on the image, what is the closest point in the graph?"""
        wMax = self.image.shape[1]
        hMax = self.image.shape[0]
        # distance formula
        distMax = pow(wMax - x, 2) + pow(hMax - y, 2)

        # find minimum distance
        for node in self.graph:
            xn, yn = decodePtName(node)
            dist = pow(xn - x, 2) + pow(yn - y, 2)
            if dist < distMax:
                wMax = xn
                hMax = yn
                distMax = dist

        # print("Closest point to {}, {} is {}, {}".format(x, y, wMax, hMax))
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
            x0, y0 = path[i]
            x1, y1 = path[i+1]

            # drawPt(img, x0, y0)
            drawLine(img, (x0, y0), (x1, y1))

        drawPt(img, x1, y1)
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

    def getCurrentCoor(self, color="yellow"):
        """Get the current IPS location of the car."""
        # validate request
        if color not in validColors:
            return 0, 0

        # request data
        URL = "http://192.168.1.8:8080/{}".format(color)
        r = rget(url=URL)

        # extract data
        coorString = r.text
        coordinates = coorString.split()
        latitude = float(coordinates[0])
        longitude = float(coordinates[1])

        return latitude, longitude


def pointClick(event, x, y, flags, params):

    if event == cv.EVENT_LBUTTONUP:
        img = np.copy(params[0])

        # find nearest feature
        ft, dist = features.findClosestFeature(x, y)
        print("{} at {}, {} pixels away".format(ft.name, ft.center, int(dist)))

        # visualize
        drawPt(img, x, y)
        drawPt(img, ft.center[0], ft.center[1])
        cv.imshow("features", img)


def testFeatureFinding(ips):
    cv.namedWindow("features")
    cv.setMouseCallback("features", pointClick, param=(ips.image,))
    cv.imshow("features", ips.image)

    while True:
        key = cv.waitKey(0) & 0xFF

        if key == ord('q'):
            break


def testPathFinding(ips):
    cv.namedWindow("path")
    p0 = 300, 520
    p1 = 700, 1450
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
    # displayRouteImg("path", ips.displayDirectedGraph())

    testPathFinding(ips)
    # testFeatureFinding(ips)

    cv.destroyAllWindows()


if __name__ == '__main__':
    main()
