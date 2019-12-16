import cv2
import numpy as np
import os


def detectColor(box):
    height = box.shape[0]

    kSz = 7
    area = box.shape[0] * box.shape[1]
    # hsv = cv2.cvtColor(box, cv2.COLOR_BGR2HSV)
    # cv2.imshow("hsv", box)
    lower_green = np.array([0, 0, 0], dtype=np.uint8)
    upper_green = np.array([100, 255, 5], dtype=np.uint8)
    mask_green = cv2.inRange(box, lower_green, upper_green)
    blur_green = cv2.GaussianBlur(mask_green, (kSz, kSz), 0)
    px_count = len((np.where(blur_green > 0))[0])
    print("{} out of {} green pixels detected".format(px_count, area))


    lower_red = np.array([0, 0, 20], dtype=np.uint8)
    upper_red = np.array([10, 0, 255], dtype=np.uint8)
    mask_red = cv2.inRange(box, lower_red, upper_red)
    blur_red = cv2.GaussianBlur(mask_red, (kSz, kSz), 0)
    px_count_red = len((np.where(blur_red > 0))[0])
    print("{} out of {} red pixels detected".format(px_count_red, area))
    cv2.imshow("frame", box)
    cv2.imshow("frame_red", blur_red)
    cv2.imshow("frame_green", blur_green)
    cv2.waitKey(0)
    if px_count_red > (area / 10):
        return "red"
    elif px_count > (area / 12):
        return "green"
    else:
        return "red"

    # cv2.imshow("green detect", blur_green)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()


def mainTest():
    imgs = list()

    # imgDir = os.path.abspath("../../testimages/trafficLights/slices")
    # imgDir = os.path.abspath("../../testimages/trafficLights/slices")
    vidcap = cv2.VideoCapture('../systemStructure/yoloVideo.avi')
    success, image = vidcap.read()
    count = 0
    while success:
        success, image = vidcap.read()
        count += 1
        imgs.append(image)
    print(count)
    # imgList = os.listdir(imgDir)
    # imgs = sorted([os.path.join(imgDir, x) for x in imgList])

    for i in imgs:
        print(i)
        # testImage = cv2.imread(i)
        print(type(i))
        color = detectColor(i)

        print(color)


if __name__ == '__main__':
    mainTest()
