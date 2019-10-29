#!/usr/bin/python3
import json
import os
import sys
from requests import get as rget
import cv2 as cv
from cv2 import imread, imshow, waitKey
import numpy as np
import networkx as nx

sys.path.append(os.getcwd())
from digraph import (drawLine, drawPt, getPtName, decodePtName)


##############################################################################
# The x,y coordinate system for the Indoor Positioning System (IPS)
# is defined to have the origin in the upper right corner of the image.
# The x-axis goes down the right side of the image, and the y-axis along
# the top.
#
# In this file, we define features of the map in terms of their pixel
# coordinates.
##############################################################################

stopSign0  = [ (148,    5), (141,  126), (266,  124), (267,    3) ]
stopLine0  = [ (303,    3), (303,   58) ]
stopLine1  = [ (207,  161), (270,  161) ]
crosswalk0 = [ (268,   10), (266,  125), (291,  124), (292,    9) ]
crosswalk1 = [ (151,  126), (151,  151), (267,  151), (266,  125) ]

stopSign1  = [ (797, 1441), (796, 1566), (919, 1567), (925, 1440) ]
stopLine2  = [ (791, 1401), (850, 1400) ]
stopLine3  = [ (754, 1512), (754, 1567) ]
crosswalk2 = [ (795, 1417), (796, 1441), (912, 1439), (912, 1415) ]
crosswalk3 = [ (771, 1442), (769, 1558), (796, 1558), (796, 1441) ]

fourWayStop= [ (447,  636), (356,  727), (360,  770), (452,  860),
               (490,  862), (582,  767), (582,  727), (489,  635) ]
stopLine4  = [ (366,  650), (324,  694) ]
stopLine5  = [ (373,  847), (418,  894) ]
stopLine6  = [ (567,  848), (613,  801) ]
stopLine7  = [ (524,  601), (565,  643) ]
crosswalk4 = [ (421,  613), (334,  704), (356,  727), (447,  636) ]
crosswalk5 = [ (360,  770), (336,  795), (426,  885), (452,  860) ]
crosswalk6 = [ (582,  767), (490,  862), (516,  888), (605,  792) ]
crosswalk7 = [ (489,  635), (582,  727), (604,  702), (514,  610) ]

validColors = ["Green", "Red", "Purple", "Light Blue", "Yellow"]


def displayRouteImg(name, img):
    """Resize image to fit the screen."""
    width = img.shape[1]
    height = img.shape[0]
    targetHeight = 960
    r = targetHeight / float(height)
    targetWidth = int(width * r)

    sz = (targetWidth, targetHeight)
    smaller = cv.resize(img, sz, interpolation=cv.INTER_AREA)

    cv.imshow(name, smaller)
    while True:
        key = cv.waitKey(0) & 0xFF
        if key == ord('q'):
            break


def findClosestFeature(x, y):
    """Given a point on the image, what is the closest feature on the road?"""
    pass


class IPS(object):
    """Implements an interface with the IPS, as well as path finding algorithms."""

    def __init__(self):
        # we're using a directed graph
        self.graph = nx.DiGraph()
        # load in the data from the graph file
        with open("./graph.json", 'r') as jf:
            self.graphDict = json.load(jf)
            self.graph = nx.from_dict_of_dicts(self.graphDict, create_using=self.graph)
        # also keep the image around
        image_path = "./Global.jpg"
        self.image = imread(image_path)

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
        """Find the shortest point between two points on the graph."""
        # find the points in the graph that is closest to what we are given
        c0 = self.findClosestGraphPoint(x0, y0)
        c1 = self.findClosestGraphPoint(x1, y1)
        # the name of the node in the graph
        n0 = getPtName(*c0)
        n1 = getPtName(*c1)

        path = nx.shortest_path(self.graph, source=n0, target=n1)
        return path

    def displayPath(self, path):
        """path is a list of node names in the graph.
        This is great because we have encoded the coordinates in the node names.
        """
        img = np.copy(self.image)
        for i in range(len(path)-1):
            x0, y0 = decodePtName(path[i])
            x1, y1 = decodePtName(path[i+1])

            drawPt(img, x0, y0)
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


def main():
    ips = IPS()
    # cv.namedWindow("graph")
    cv.namedWindow("path")

    # displayRouteImg("graph", ips.displayDirectedGraph())

    p0 = 110, 120
    p1 = 700, 1550
    RED = (0, 0, 255)
    path = ips.findPath(*p0, *p1)
    print(path)

    pathImg = ips.displayPath(path)
    drawPt(pathImg, *p0, color=RED)
    drawLine(pathImg, p0, decodePtName(path[0]), color=RED)
    drawPt(pathImg, *p1, color=RED)
    drawLine(pathImg, decodePtName(path[-1]), p1, color=RED)

    displayRouteImg("path", pathImg)

    cv.destroyAllWindows()


if __name__ == '__main__':
    main()
