import sys
from code.systemStructure.pollers import Pollers
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
    # print(self.frame_laneDetectQ.get())
    framePollerProcess.join()

    # IPS poller process setup and start

    # Lane detector process setup and start

    # Stop detector process setup and start

    # Emergency stop process setup and start

    # Route manager process setup and start



if __name__ == "__main__":
    main()