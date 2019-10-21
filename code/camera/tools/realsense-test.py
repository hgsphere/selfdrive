#!/usr/bin/python3

import pyrealsense2 as rs
import numpy as np
import cv2 as cv
import time
import sys
import os


def main():
    config = rs.config()
    shape = (640, 480)
    shape_rgb = (424, 240)
    frame_rate = 30
    frame_rate_rgb = 60
    
    config.enable_stream(rs.stream.depth, shape[0], shape[1], rs.format.z16, frame_rate)
    config.enable_stream(rs.stream.color, shape_rgb[0], shape_rgb[1], rs.format.bgr8, frame_rate_rgb)
    pipeline = rs.pipeline()
    print("pipeline created")

    # make a video writer
    fourcc = cv.VideoWriter_fourcc(*"MJPG")
    outpath_rgb = os.path.join(os.getcwd(), "output-rgb-low.avi")
    outpath_dep = os.path.join(os.getcwd(), "output-depth-low.avi")
    writer_rgb = cv.VideoWriter(outpath_rgb, fourcc, frame_rate_rgb,
            (shape_rgb[0], shape_rgb[1]), True)
    writer_dep = cv.VideoWriter(outpath_dep, fourcc, frame_rate,
            (shape[0], shape[1]), True)

    try:
        # Create a context object. This object owns the handles to all connected realsense devices
        pipeline.start(config)
        print("pipeline started")
        sys.stdout.flush()

        #while True:
        for i in range(60):
            time.sleep(0.01)
            # This call waits until a new coherent set of frames is available on a device
            # Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
            frames = pipeline.wait_for_frames()
            depth = frames.get_depth_frame()
            color = frames.get_color_frame()
            if not depth: 
                continue

            colorData = np.asanyarray(color.get_data())
            depthData = np.asanyarray(depth.get_data())

            # Render images
            depth_colormap = np.asanyarray(cv.applyColorMap(
                    cv.convertScaleAbs(depthData, alpha=0.03), cv.COLORMAP_JET))
            
            # cv.imwrite(outpath_rgb, colorData)
            # cv.imwrite(outpath_dep, depth_colormap)
            writer_rgb.write(colorData)
            writer_dep.write(depth_colormap)

        exit(0)
    #except rs.error as e:
    #    # Method calls agaisnt librealsense objects may throw exceptions of type pylibrs.error
    #    print("pylibrs.error was thrown when calling %s(%s):\n", % (e.get_failed_function(), e.get_failed_args()))
    #    print("    %s\n", e.what())
    #    exit(1)
    except Exception as e:
        print(e)
        pass

    writer_rgb.release()
    writer_dep.release()

    print("We're all done now!")
    sys.stdout.flush()


if __name__ == "__main__":
    main()
