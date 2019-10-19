#!/usr/bin/python3

import sys
from code.systemStructure.pollers import Pollers
from code.car.LaneDetector import lanefollower
from code.systemStructure.RouteManager import RouteManager
from code.systemStructure.EmergencyStopDetector import EmergencyStopDetector
import multiprocessing as mp


class SystemManager():
    def __init__(self):
        self.stuff = None
        self.frame_laneDetectQ = mp.Queue()
        self.frame_emergencyStopQ = mp.Queue()
        self.frame_stopDetectQ = mp.Queue()
        self.IPS_routeManagerQ = mp.Queue()
        self.laneDetect_routeManagerQ = mp.Queue()
        self.emergencyStop_routeManagerQ = mp.Queue()
        self.stopDetect_routeManagerQ = mp.Queue()
        self.poller = Pollers(False)
        self.lnFollower = lanefollower()
        self.routeManager = RouteManager()
        self.emStop = EmergencyStopDetector()

    def initializeSystem(self):
        print("Initializing System")
        print("Starting FramePoller")
        print("Starting IPSPoller")
        print("System Initialization Complete")

    def shutdownSystem(self):
        print("Shutting down system")

    def main(self):
        if sys.argv[1] is True:
            print("debugging enabled")
        ctx = mp.get_context('spawn')
        #frame poller process setup and start
        framePollerProcess = ctx.Process(target=self.poller.pollFrame, args=(self.frame_laneDetectQ, self.frame_emergencyStopQ, self.frame_stopDetectQ))
        framePollerProcess.start()
        # IPS poller process setup and start

        # Lane detector process setup and start
        laneDetectorProcess = ctx.Process(target=self.lnFollower.getCorrectionAngle, args=(self.frame_laneDetectQ, self.laneDetect_routeManagerQ))
        laneDetectorProcess.start()

        # Stop detector process setup and start
        stopDetectorProcess = ctx.Process(target=self.lnFollower.getCrosswalk, args=(self.frame_stopDetectQ, self.stopDetect_routeManagerQ))
        stopDetectorProcess.start()

        # Emergency stop process setup and start
        emergencyStopProcess = ctx.Process(target=self.emStop.detectStop, args=(self.frame_emergencyStopQ, self.emergencyStop_routeManagerQ))
        emergencyStopProcess.start()

        # Route manager process setup and start
        routeManagerProcess = ctx.Process(target=self.routeManager.runSupervisorStateMachine, args=(self.laneDetect_routeManagerQ, self.stopDetect_routeManagerQ, self.emergencyStop_routeManagerQ))
        routeManagerProcess.start()

        # wait for everything to complete
        framePollerProcess.join()
        emergencyStopProcess.join()
        laneDetectorProcess.join()
        stopDetectorProcess.join()
        routeManagerProcess.join()


def wrapperMain():
    sysM = SystemManager()
    sysM.main()


if __name__ == "__main__":
    wrapperMain()
