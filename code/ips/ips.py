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

hardTurn0 = [ (30, 1274), (126, 1278), (125, 1180), (23, 1179) ]
stopLine0 = [ (155, 1278), (155, 1232) ]
stopLine1 = [ (77, 1148), (129, 1149) ]
crosswalk0 = [ (127, 1272), (146, 1272), (146, 1179), (126, 1179) ]
crosswalk1 = [ (31, 1176), (126, 1279), (126, 1158), (32, 1157) ]

hardTurn1 = [ (596, 127), (658, 127), (658, 24), (596, 23) ]
stopLine2 = [ (562, 70), (558, 25) ]
stopLine3 = [ (590, 158), (638, 160) ]
crosswalk2 = [ (573, 126), (595, 126), (594, 33), (575, 32) ]
crosswalk3 = [ (595, 145), (656, 148), (657, 127), (595, 126) ]

fourWayStop = [ (293, 790), (410, 685), (303, 571), (185, 680) ]
stopLine4 = [ (215, 756), (183, 721) ]
stopLine5 = [ (227, 596), (265, 563) ]
stopLine6 = [ (390, 603), (425, 641) ]
stopLine7 = [ (381, 768), (346, 801) ]
crosswalk4 = [ (190, 712), (261, 787), (282, 770), (213, 696) ]
crosswalk5 = [ (215, 660), (292, 593), (274, 570), (195, 639) ]
crosswalk6 = [ (326, 590), (398, 668), (418, 649), (348, 570) ]
crosswalk7 = [ (397, 704), (319, 772), (338, 793), (417, 722) ]

validColors = ["Green", "Red", "Purple", "Light Blue", "Dark Blue"]


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
