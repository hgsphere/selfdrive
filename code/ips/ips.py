import os
import sys
from requests import get as rget
from cv2 import imread, imshow, waitKey

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


def showGlobalImage():
    path = "./Global.jpeg"
    image = imread(path)
    imshow("global", image)
    waitKey(0)


def getCoor(color):
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
    showGlobalImage()


if __name__ == '__main__':
    main()
