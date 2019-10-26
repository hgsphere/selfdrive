from multiprocessing import Queue
import pyrealsense2 as rs
import numpy as np
import cv2 as cv
import time
import sys
import os
import multiprocessing, logging

#This prints in threads
#logger = multiprocessing.log_to_stderr()
#logger.setLevel(logging.INFO)
#logger.warning('doomed')

class Pollers():
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
        # shape_depth = (640, 480)
        self.shape_rgb = (424, 240)

        # shape_depth = (480,270)
        frame_rate = 30
        frame_rate_rgb = 60
        # resolution = (640, 480)
        # resolution = shape
        # outPath = 'test.avi'
        # self.out = cv.VideoWriter(outPath, cv.VideoWriter_fourcc('M','J','P','G'), frame_rate, resolution)

        config.enable_stream(rs.stream.depth, shape[0], shape[1], rs.format.z16, frame_rate)
        # config.enable_stream(rs.stream.depth, shape_depth[0], shape_depth[1], rs.format.z16, frame_rate)
        config.enable_stream(rs.stream.color, self.shape_rgb[0], self.shape_rgb[1], rs.format.bgr8, frame_rate_rgb)

        self.profile = self.pipeline.start(config)
        time.sleep(1)  # wait for warmup
        depth_sensor = self.profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        clipping_distance_in_meters = 0.3
        self.clipping_distance = clipping_distance_in_meters / depth_scale

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

        # make a video writer
        # fourcc = cv.VideoWriter_fourcc(*"MJPG")
        # outpath_rgb = os.path.join(os.getcwd(), "output-rgb.avi")
        # outpath_dep = os.path.join(os.getcwd(), "output-depth.avi")
        # writer_rgb = cv.VideoWriter(outpath_rgb, fourcc, frame_rate,
        #                             (shape[0], shape[1]), True)
        # writer_dep = cv.VideoWriter(outpath_dep, fourcc, frame_rate,
        #                             (shape[0], shape[1]), True)

        try:
            # Create a context object. This object owns the handles to all connected realsense devices

            # for i in range(60):
            # time.sleep(0.01)
            # This call waits until a new coherent set of frames is available on a device
            # Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
            frames = self.pipeline.wait_for_frames(timeout_ms=10000)
            # if not res:
            #     return None, None
            depth = frames.get_depth_frame()
            color = frames.get_color_frame()

            #print("Next frame available")
            colorData = np.asanyarray(color.get_data())
            depthData = np.asanyarray(depth.get_data())
            #print("pre process depth data")
            # depthFloat = self.processDepthFrame(depthData, depth.width, depth.height)

            # print(toLaneDetectQ.qsize())
            #print("filling queues...")
            #logger.error('Here I am')

            # toLaneDetectQ.put(colorData)
            # toStopDetectQ.put(colorData)
            # toEmergencyStopQ.put(depthFloat)
            return colorData, depthData

#                print("filled queues")

                # Render images
                # depth_colormap = np.asanyarray(cv.applyColorMap(
                #     cv.convertScaleAbs(depthData, alpha=0.03), cv.COLORMAP_JET))

                # cv.imwrite(outpath_rgb, colorData)
                # depth_colormap = np.asanyarray(cv.applyColorMap(
                #     cv.convertScaleAbs(depthData, alpha=0.03), cv.COLORMAP_JET))

                # cv.imwrite(outpath_rgb, colorData)
                # depth_colormap = np.asanyarray(cv.applyColorMap(
                #     cv.convertScaleAbs(depthData, alpha=0.03), cv.COLORMAP_JET))

                # cv.imwrite(outpath_rgb, colorData)
                # cv.imwrite(outpath_dep, depth_colormap)
                # writer_rgb.write(colorData)
                # writer_dep.write(depth_colormap)
            #raise Exception
        # except rs.error as e:
        #    # Method calls agaisnt librealsense objects may throw exceptions of type pylibrs.error
        #    print("pylibrs.error was thrown when calling %s(%s):\n", % (e.get_failed_function(), e.get_failed_args()))
        #    print("    %s\n", e.what())
        #    exit(1)
        except Exception as e:
            # print('Did I make it')
            sys.stdout.flush()
            sys.stderr.write("exception hit\n")
            sys.stderr.write(str(e))
            #logger.error(e)
            raise e
