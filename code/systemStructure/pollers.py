import pyrealsense2 as rs
import numpy as np
import cv2 as cv
import time
import sys

# This prints in threads
# logger = multiprocessing.log_to_stderr()
# logger.setLevel(logging.INFO)
# logger.warning('doomed')


class Pollers(object):
    def __init__(self, debugging=False):
        self.debugging = debugging
        self.clipping_distance = None
        self.profile = None

        self.config_frames_pipeline()

    def config_frames_pipeline(self):
        rs.log_to_console(rs.log_severity.info)
        self.pipeline = rs.pipeline()
        config = rs.config()
        shape = (640, 480)
        self.shape_rgb = (424, 240)

        # shape_depth = (480,270)
        frame_rate = 30
        frame_rate_rgb = 60
        # resolution = (640, 480)
        # resolution = shape

        config.enable_stream(rs.stream.depth, shape[0], shape[1], rs.format.z16, frame_rate)
        config.enable_stream(rs.stream.color, self.shape_rgb[0], self.shape_rgb[1], rs.format.bgr8, frame_rate_rgb)

        self.profile = self.pipeline.start(config)
        time.sleep(1)  # wait for warmup
        depth_sensor = self.profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        clipping_distance_in_meters = 0.3
        self.clipping_distance = clipping_distance_in_meters / depth_scale

    def getClippingDistance(self):
        if self.profile is not None:
            return self.clipping_distance
        else:
            return None

    def processDepthFrame(self, depthData, width, height):
        depthFloat = np.zeros(depthData.shape, dtype=np.float32)

        # threshold the depth image
        grey_color = 153
        for i in range(width):
            for j in range(height):
                distance = depthData[j, i]
                if distance > self.clipping_distance:
                    depthFloat[j, i] = grey_color
                else:
                    depthFloat[j, i] = distance

        gauss = cv.GaussianBlur(depthFloat, (5, 5), -1)
        return gauss

    def pollFrame(self):
        # retrieve frame from camera and place it in the input queue to the LaneDetector Data Interpreter

        try:
            # Create a context object. This object owns the handles to all connected realsense devices

            # This call waits until a new coherent set of frames is available on a device
            # Calls to get_frame_data(...) and get_frame_timestamp(...)
            #  on a device will return stable values until wait_for_frames(...) is called
            frames = self.pipeline.wait_for_frames(timeout_ms=10000)
            # if not res:
            #     return None, None
            depth = frames.get_depth_frame()
            color = frames.get_color_frame()

            # print("Next frame available")
            colorData = np.asanyarray(color.get_data())
            depthData = np.asanyarray(depth.get_data())
            # print("pre process depth data")
            # depthFloat = self.processDepthFrame(depthData, depth.width, depth.height)

            # logger.error('Here I am')

            return colorData, depthData

        except Exception as e:
            sys.stdout.flush()
            sys.stderr.write("exception hit\n")
            sys.stderr.write(str(e))
            # logger.error(e)
            raise e
