from multiprocessing import Queue
import pyrealsense2 as rs
import numpy as np
import cv2 as cv
import time
import sys
import os
import multiprocessing, logging
logger = multiprocessing.log_to_stderr()
logger.setLevel(logging.INFO)
logger.warning('doomed')

class Pollers():
    def __init__(self, debugging):
        self.debugging = debugging
        self.clipping_distance = None

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

    def pollFrame(self, toLaneDetectQ, toEmergencyStopQ, toStopDetectQ):
        # retrieve frame from camera and place it in the input queue to the LaneDetector Data Interpreter
        config = rs.config()
        shape = (640, 480)
        frame_rate = 30

        config.enable_stream(rs.stream.depth, shape[0], shape[1], rs.format.z16, frame_rate)
        config.enable_stream(rs.stream.color, shape[0], shape[1], rs.format.bgr8, frame_rate)
        pipeline = rs.pipeline()
        print("pipeline created")

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
            profile = pipeline.start(config)
            depth_sensor = profile.get_device().first_depth_sensor()
            depth_scale = depth_sensor.get_depth_scale()
            clipping_distance_in_meters = 0.3
            self.clipping_distance = clipping_distance_in_meters / depth_scale

            print("pipeline started")
            sys.stdout.flush()

            while True:
                # for i in range(60):
                time.sleep(0.01)
                # This call waits until a new coherent set of frames is available on a device
                # Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
                frames = pipeline.wait_for_frames()
                depth = frames.get_depth_frame()
                color = frames.get_color_frame()
                if not depth:
                    continue

                print("Next frame available")
                colorData = np.asanyarray(color.get_data())
                depthData = np.asanyarray(depth.get_data())
                print("pre process depth data")
                depthFloat = self.processDepthFrame(depthData, depth.width, depth.height)

                print(toLaneDetectQ.qsize())
                print("filling queues...")
                logger.error('Here I am')
                toLaneDetectQ.put(colorData)
                toStopDetectQ.put(colorData)
                toEmergencyStopQ.put(depthFloat)
                print("filled queues")

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
            print('Did I make it')
            sys.stdout.flush()
            sys.stderr.write("exception hit\n")
            sys.stderr.write(e)
            logger.error(e)
            raise e
