#!/usr/bin/python3 

import sys
import os

sys.path.append(os.path.abspath("../ips"))
sys.path.append(os.path.abspath("../systemStructure"))
sys.path.append(os.path.abspath("../car"))

from ImageProcessor import imageprocessor
from RouteManager import RouteManager
from ips import *
import multiprocessing as mp
import threading
from queue import Queue


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
        # create the objects
        self.imgProc = imageprocessor()
        self.routeManager = RouteManager()

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
        try:
            ctx = mp.get_context('fork')
            # IPS poller process setup and start
            ipsPollProcess = ctx.Process(target=pollCoordinates, args=(self.IPS_routeManagerQ,), name="IPSPoller")
            ipsPollProcess.start()
            # Lane detector process setup and start
            imageProcessorProcess = ctx.Process(target=self.imgProc.runImageProcessing,
                                                args=(self.laneDetect_routeManagerQ,
                                                      self.stopDetect_routeManagerQ,
                                                      self.emergencyStop_routeManagerQ),
                                                name="ImageProcessor")
            imageProcessorProcess.start()

            # Route manager process setup and start
            routeManagerProcess = ctx.Process(target=self.routeManager.runSupervisorStateMachine,
                                              args=(self.laneDetect_routeManagerQ,
                                                    self.stopDetect_routeManagerQ,
                                                    self.emergencyStop_routeManagerQ,
                                                    self.IPS_routeManagerQ),
                                              name="RouteManager")
            routeManagerProcess.start()
    
            print("All processes started")
            # wait for everything to complete
            imageProcessorProcess.join()
            ipsPollProcess.join()
            routeManagerProcess.join()

        except Exception as e:
            print("Error in system manager")
            print(e)
            raise e
     


def wrapperMain():
    sysM = SystemManager()
    sysM.main()


if __name__ == "__main__":
    wrapperMain()
