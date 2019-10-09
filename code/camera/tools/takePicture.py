import os
import sys
import glob
import argparse
import cv2 as cv


def parseArgs():
    ap = argparse.ArgumentParser(description="takes a single picture using the RealSense camera")
    ap.add_argument('-o', '--output', required=True,
            help="path to save image to")

    args = ap.parse_args()
    return args


def getCameraInput():
    cameraDevice = "/dev/video3"

    if os.path.exists(cameraDevice):
        vs = cv.VideoCapture(cameraDevice, cv.CAP_V4L)
    else:
        videoNames = glob.glob("/dev/video*")
        if videoNames is None:
            return None
        highestName = sorted(videoNames)[-1]
        vs = cv.VideoCapture(highestName, cv.CAP_V4L)

    return vs


def main():
    args = parseArgs()

    vs = getCameraInput()
    if vs is None:
        print("Error opening camera!", file=sys.stderr)

    (grabbed, frame) = vs.read()
    # make sure we get a frame
    while not grabbed:
        (grabbed, frame) = vs.read()

    # write out
    cv.imwrite(args.output, frame)

    # release camera input
    vs.release()


if __name__ == '__main__':
    main()
