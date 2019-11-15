import mxnet as mx
from gluoncv import model_zoo, utils
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
        '''resize image with unchanged aspect ratio using padding'''
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
        frameRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # from gluoncv import data
        yolo_image = Image.fromarray(frameRGB, 'RGB')
        x, img = self.load_test(yolo_image, short=416)

        # parse stuff
        class_IDs, scores, bounding_boxs = self.net(x.copyto(self.device))

        # Convert to numpy arrays, then to lists
        class_IDs = class_IDs.asnumpy().tolist()
        scores = scores.asnumpy().tolist()
        bounding_boxs = bounding_boxs.asnumpy()

        maxConfidence = 0.0
        boxSlice = None

        # iterate through detected objects
        for i in range(len(class_IDs[0])):
            current_score = (scores[0][i])[0]
            if current_score > self.confidence:
                current_class_id = self.net.classes[int((class_IDs[0][i])[0])]

                if current_score > maxConfidence and current_class_id == "traffic light":
                    maxConfidence = current_score
                    current_bb = bounding_boxs[0][i]

                    # get only the part of the image with the stoplight in it
                    pt0 = (int(current_bb[0]), int(current_bb[1]))
                    pt1 = (int(current_bb[2]), int(current_bb[3]))
                    boxSlice = img[pt0[1]:pt1[1], pt0[0]:pt1[0]]

        if boxSlice is not None:
            boxSlice = cv2.cvtColor(boxSlice, cv2.COLOR_BGR2RGB)
            cv2.imshow("boxSlice", boxSlice)
            cv2.waitKey(0)

        gc.collect()
        return boxSlice

    def detectColor(self, box):
        return "red"

    def detectTrafficLight(self, frame):
        # can read frames from paths
        if isinstance(frame, str):
            frame = cv2.imread(frame)

        # get the bounding box
        boxSlice = self.runYolo(frame)

        if boxSlice is not None:
            # look for a color
            self.detectColor(boxSlice)


# TODO: this is not yet implemented
def runYoloDetector(inputQueue):
    tld = trafficLightDetector()

    while True:
        nextFrame = inputQueue.get()
        color = tld.detectTrafficLight(nextFrame)


def mainTest():
    # test if the color detector works
    tld = trafficLightDetector()

    testImage = cv2.imread("../../testimages/trafficLights/red.png")
    color = tld.detectColor(testImage)

    print(color)


if __name__ == '__main__':
    mainTest()
