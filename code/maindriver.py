#!/usr/bin/python3

# This is the main driver for the self-driving car project.
# It interfaces with each of the elements of the code, including
#  frame acquisition, route planning, localization, etc.

import os
import sys

sys.path.append(os.path.abspath("./utils/"))
sys.path.append(os.path.abspath("./camera/"))

from camera.calibrate import getHomographyMatrix


def main():
    pass


if __name__ == '__main__':
    main()
