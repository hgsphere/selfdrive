#!/usr/bin/python3 

import sys
import os
import multiprocessing as mp
import threading
from queue import Queue

sys.path.append(os.path.abspath("../ips"))
sys.path.append(os.path.abspath("../systemStructure"))
sys.path.append(os.path.abspath("../car"))
sys.path.append(os.path.abspath("../yolo"))

from ImageProcessor import imageprocessor
from RouteManager import RouteManager
from ips import *
from trafficLight import runYoloDetector


class SystemManager(object):
    def __init__(self):
        self.stuff = None
        # queues for the Image Processor
        self.frame_laneDetectQ = Queue(maxsize=60)
        self.frame_emergencyStopQ = Queue(maxsize=60)
        self.frame_stopDetectQ = Queue(maxsize=60)
        # queues that the route manager pulls from
        self.IPS_routeManagerQ = mp.Queue(maxsize=1)
        self.laneDetect_routeManagerQ = mp.Queue(maxsize=60)
        self.emergencyStop_routeManagerQ = mp.Queue(maxsize=60)
        self.stopDetect_routeManagerQ = mp.Queue(maxsize=60)
        # For transfering data to/from Yolo detector
        self.yolo_pipe = mp.Pipe()                  # transfer the next frame
        self.yolo_ready_flag = mp.Value('i', 0)     # detector is ready for another frame
        self.yolo_green_flag = mp.Value('i', 0)     # green light detected
        self.stop_now_flag = mp.Value('i', 0)       # traffic light is close, stop
        # create the objects
        self.imgProc = imageprocessor()
        self.routeManager = RouteManager()

        # fix YOLO thing
        os.putenv("MXNET_CUDNN_AUTOTUNE_DEFAULT", "0")

    def initializeSystem(self):
        print("Initializing System")
        print("Starting FramePoller")
        print("Starting IPSPoller")
        print("System Initialization Complete")

    def shutdownSystem(self):
        print("Shutting down system")

    def main(self):
        if len(sys.argv) > 1 and sys.argv[1] is True:
            print("debugging enabled")

        # set up stuff for good termination later
        ipsPollProcess = None
        imageProcessorProcess = None
        routeManagerProcess = None
        yoloDetectorProcess = None

        try:
            ctx = mp.get_context('fork')
            # IPS poller process setup and start
            lat = mp.Value('d', 0.0)
            lon = mp.Value('d', 0.0)
            ipsPollProcess = ctx.Process(target=pollCoordinates, args=(lat,lon), name="IPSPoller")
            ipsPollProcess.start()
            # Lane detector process setup and start
            imageProcessorProcess = ctx.Process(target=self.imgProc.runImageProcessing,
                                                args=(self.laneDetect_routeManagerQ,
                                                      self.stopDetect_routeManagerQ,
                                                      self.emergencyStop_routeManagerQ,
                                                      self.yolo_pipe,
                                                      self.yolo_ready_flag),
                                                name="ImageProcessor")
            imageProcessorProcess.start()

            # Route manager process setup and start
            routeManagerProcess = ctx.Process(target=self.routeManager.runSupervisorStateMachine,
                                              args=(self.laneDetect_routeManagerQ,
                                                    self.stopDetect_routeManagerQ,
                                                    self.emergencyStop_routeManagerQ,
                                                    #self.IPS_routeManagerQ),
                                                    lat,lon,
                                                    self.yolo_green_flag,
                                                    self.stop_now_flag),
                                              name="RouteManager")
            routeManagerProcess.start()

            # yolo detector process
            yoloDetectorProcess = ctx.Process(target=runYoloDetector,
                                              args=(self.yolo_pipe,
                                                    self.yolo_ready_flag,
                                                    self.yolo_green_flag,
                                                    self.stop_now_flag),
                                              name="YoloDetector")
            yoloDetectorProcess.start()
    
            print("All processes started")
            # wait for everything to complete
            imageProcessorProcess.join()
            ipsPollProcess.join()
            routeManagerProcess.join()
            yoloDetectorProcess.join()

        except KeyboardInterrupt as ke:
            print("Shutting down")
            if imageProcessorProcess is not None:
                imageProcessorProcess.terminate()
            if ipsPollProcess is not None:
                ipsPollProcess.terminate()
            if routeManagerProcess is not None:
                routeManagerProcess.terminate()
            if yoloDetectorProcess is not None:
                yoloDetectorProcess.terminate()
            raise ke
        except Exception as e:
            print("Error in system manager")
            print(e)
            raise e
     


def wrapperMain():
    sysM = SystemManager()
    sysM.main()


if __name__ == "__main__":
    wrapperMain()
