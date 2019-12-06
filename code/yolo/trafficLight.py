#!/usr/bin/python3

import mxnet as mx
from gluoncv import model_zoo, utils
from time import perf_counter
from PIL import Image
import numpy as np
import cv2
import gc
import os


class trafficLightDetector(object):
    def __init__(self):
        # Implement YOLOv3MXNet
        self.net = model_zoo.get_model('yolo3_mobilenet1.0_coco', pretrained=True)
        # Set device to GPU
        self.device = mx.gpu()
        self.net.collect_params().reset_ctx(self.device)
        # detected bounding boxes
        self.confidence = 0.55
        self.video_writer = None
        self.TAKE_VIDEO = True
        if self.TAKE_VIDEO:
            self.init_video_writer()
        # flag to tell the car to stop
        self.stop_now_flag = None
        # averaging the box around the light in case it loses it for one frame
        self.avgBoxPts = None

    def __del__(self):
        if self.TAKE_VIDEO:
            self.video_writer.release()

    def init_video_writer(self):
        # shape_rgb = (416, 235)
        shape_rgb = (282, 282)
        # self.y_offset = (416 - 235) // 2
        frame_rate_rgb = 3
        fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        outpath_rgb = os.path.join(os.getcwd(), "yoloVideo.avi")
        # outpath_dep = os.path.join(os.getcwd(), "output-depth-low.avi")
        self.video_writer = cv2.VideoWriter(outpath_rgb, fourcc, frame_rate_rgb,
                                       (shape_rgb[0], shape_rgb[1]), True)

    """Transforms for YOLO series."""
    def transform_test(self, imgs, mean=(0.485, 0.456, 0.406), std=(0.229, 0.224, 0.225)):
        if isinstance(imgs, mx.nd.NDArray):
            imgs = [imgs]
        for im in imgs:
            assert isinstance(im, mx.nd.NDArray), "Expect NDArray, got {}".format(type(im))

        tensors = []
        origs = []
        for img in imgs:
            orig_img = img.asnumpy().astype('uint8')
            img = mx.nd.image.to_tensor(img)

            img = mx.nd.image.normalize(img, mean=mean, std=std)

            tensors.append(img.expand_dims(0))
            origs.append(orig_img)
        if len(tensors) == 1:
            return tensors[0], origs[0]
        return tensors, origs

    def load_test(self, filenames, short=416):
        if not isinstance(filenames, list):
            filenames = [filenames]
        imgs = [self.letterbox_image(f, short) for f in filenames]
        return self.transform_test(imgs)

    # this function is from yolo3.utils.letterbox_image
    def letterbox_image(self, image, size=416):
        """resize image with unchanged aspect ratio using padding"""
        iw, ih = image.size

        scale = min(size/iw, size/ih)
        nw = int(iw*scale)
        nh = int(ih*scale)

        image = image.resize((nw, nh), Image.BICUBIC)
        new_image = Image.new('RGB', (size, size), (128, 128, 128))
        new_image.paste(image, ((size-nw)//2, (size-nh)//2))
        return mx.nd.array(np.array(new_image))

    def runYolo(self, frame):
        # some conversions
        height = frame.shape[0]
        width = frame.shape[1]
        frameRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # from gluoncv import data
        yolo_image = Image.fromarray(frameRGB, 'RGB')
        newWidth = width if width < 416 else 416
        x, img = self.load_test(yolo_image, short=newWidth)

        # parse stuff
        class_IDs, scores, bounding_boxs = self.net(x.copyto(self.device))

        # Convert to numpy arrays, then to lists
        class_IDs = class_IDs.asnumpy().tolist()
        scores = scores.asnumpy().tolist()
        bounding_boxs = bounding_boxs.asnumpy()

        interestingClasses = ["traffic light"]

        maxConfidence = 0.0
        maxBoxSize = 0
        boxSlice = None
        pt0, pt1 = None, None

        # iterate through detected objects
        for i in range(len(class_IDs[0])):
            # filter by class
            current_class_id = self.net.classes[int((class_IDs[0][i])[0])]
            if current_class_id not in interestingClasses:
                continue

            # filter by confidence
            current_score = (scores[0][i])[0]
            if current_score < self.confidence:
                continue

            # unpack
            current_bb = bounding_boxs[0][i]
            x0, y0, x1, y1 = current_bb
            curArea = (x1-x0) * (y1-y0)

            # first filter by area
            if curArea > maxBoxSize:
                maxBoxSize = curArea

                # get only the part of the image with the stoplight in it
                pt0 = (int(x0), int(y0))
                pt1 = (int(x1), int(y1))

                print("  --  Found traffic light with confidence {} --".format(current_score))
            elif curArea == maxBoxSize:
                if current_score > maxConfidence:
                    maxConfidence = current_score

                    # get only the part of the image with the stoplight in it
                    pt0 = (int(x0), int(y0))
                    pt1 = (int(x1), int(y1))
                    print("  --  Found traffic light with confidence {} --".format(current_score))

                    # see if the traffic light is at the top of the image
                    # if so, then we should probably stop
                    if pt0[0] < 5:
                        self.stop_now_flag.value = 1
                    else:
                        self.stop_now_flag.value = 0

        if pt0 is None:
            self.avgBoxPts = self.avgBoxPts
        elif self.avgBoxPts is None and pt0 is not None:
            self.avgBoxPts = np.asanyarray([*pt0, *pt1])
        elif self.avgBoxPts is not None:
            self.avgBoxPts = (self.avgBoxPts * 0.7 + np.asanyarray([*pt0, *pt1]) * 0.3).astype("int")

        if self.avgBoxPts is not None:
            x0, y0, x1, y1 = self.avgBoxPts
            boxSlice = img[y0:y1, x0:x1]
            # boxSlice = img[pt0[1]:pt1[1], pt0[0]:pt1[0]]

            boxSlice = cv2.cvtColor(boxSlice, cv2.COLOR_BGR2RGB)
            colorDetect = self.detectColor(boxSlice)
            # cv2.imshow("boxSlice", boxSlice)
            # cv2.waitKey(0)
            if self.TAKE_VIDEO and pt0 is not None:
                img2 = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                cv2.rectangle(img2, pt0, pt1, (0, 255, 0), 2)
                # cv2.imwrite("yoloImage.jpeg", img2)
                if colorDetect == "green":
                    cv2.rectangle(img2, (40, 300), (80, 340), (0, 255, 0), -1)
                    # cv2.addText(img2, "GREEN", (40, 300), cv2.FONT_HERSHEY_SIMPLEX, 1)
                self.video_writer.write(img2)
                # print("YOLO Frame information:")
                # print(type(img))
                # print(img.shape)
        else:
            colorDetect = "red"
            if self.TAKE_VIDEO:
                img2 = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                self.video_writer.write(img2)

        # gc.collect()
        return colorDetect

    def detectColor(self, box):
        # height = box.shape[0]
        kSz = 7
        area = box.shape[0] * box.shape[1]

        # hsv stuff:
        hsv_lower_green = np.array([120, 100, 47.1], dtype=np.uint8)
        hsv_upper_green = np.array([116, 64.7, 100], dtype=np.uint8)
        hsv_mask_green = cv2.inRange(box, hsv_lower_green, hsv_upper_green)
        hsv_blur_green = cv2.GaussianBlur(hsv_mask_green, (kSz, kSz), 0)
        hsv_px_count = len((np.where(hsv_blur_green > 0))[0])
        if hsv_px_count > (area / 8):
            return "green"
        else:
            return "red"

        # rgb stuff:
        # lower_green = np.array([0, 120, 0], dtype=np.uint8)
        # upper_green = np.array([100, 255, 90], dtype=np.uint8)
        # mask_green = cv2.inRange(box, lower_green, upper_green)
        # blur_green = cv2.GaussianBlur(mask_green, (kSz, kSz), 0)
        #
        # px_count = len((np.where(blur_green > 0))[0])
        #
        # if px_count > (area / 8):
        #     return "green"
        # else:
        #     return "red"

        # looking for the brightest light
        # gray = cv2.cvtColor(box, cv2.COLOR_BGR2GRAY)
        # radius = 7
        # kSz = 7
        # # blur to make it more robust
        # blur = cv2.GaussianBlur(gray, (radius, radius), 0)
        #
        # lightThreshold = 200
        # # avgVal = np.average(blur)
        # # find the coordinates of the brightest pixel
        # (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(blur)
        #
        # # print("yolo average value in bounding box =1 {}".format(avgVal))
        # # print("yolo max value = {}".format(maxVal))
        #
        # # if the average value is below a threshold, then don't do detection
        # if maxVal < lightThreshold:
        #     return "red"
        #
        # # display
        # # image = box.copy()
        # # cv2.circle(image, maxLoc, radius, (255, 0, 0), 2)
        # # cv2.imshow("spot", image)
        # # cv2.waitKey(0)
        #
        # # look at the bottom quarter of the image
        # # this can be adjusted as needed
        # threshold = height * (3.0/4.0)
        # if maxLoc[1] > threshold:
        #     return "green"
        # else:
        #     return "red"


    def detectTrafficLight(self, frame):
        # can read frames from paths
        if isinstance(frame, str):
            frame = cv2.imread(frame)

        # get the bounding box
        return self.runYolo(frame)

        # if boxSlice is not None:
        #     # look for a color
        #     return self.detectColor(boxSlice)
        # else:
        #     return "red"


def runYoloDetector(yolo_pipe, readyFlag, greenFlag, stop_now_flag):
    pipe_output, pipe_input = yolo_pipe
    pipe_input.close()      # only reading

    # TODO: check that environment variable is set correctly

    readyFlag.value = 1

    tld = trafficLightDetector()
    tld.stop_now_flag = stop_now_flag
    tld.stop_now_flag.value = 0

    tStart = perf_counter()

    while True:
        # get the next frame
        nextFrame = pipe_output.recv()
        # set the ready flag high
        readyFlag.value = 1

        # do the detection
        color = tld.detectTrafficLight(nextFrame)
        tEnd = perf_counter()
        print(" - yolo - tDiff = {}".format(tEnd - tStart))

        # set the output flag
        if color == 'green':
            greenFlag.value = 1
        else:
            greenFlag.value = 0
        tStart = perf_counter()


def mainTest():
    # test if the color detector works
    tld = trafficLightDetector()

    imgDir = os.path.abspath("../../testimages/trafficLights/slices")
    imgList = os.listdir(imgDir)
    imgs = sorted([os.path.join(imgDir, x) for x in imgList])

    for i in imgs:
        print(i)
        testImage = cv2.imread(i)
        color = tld.detectColor(testImage)

        print(color)


if __name__ == '__main__':
    mainTest()
