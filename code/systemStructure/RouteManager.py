import os
import sys

sys.path.append(os.path.abspath("../car"))
from asyncDrive import asyncDrive


class RouteManager():

    def __init__(self):
        self.laneDetectQ = None
        self.stopDetectQ = None
        self.emergencyStopQ = None

        self.angle = None
        self.crosswalk = None
        self.emergencyStop = None

        self.States = {
            "Init": 0,
            "Stop": 1,
            "Lane_Follow": 2,
            "Force_Forward": 3,
            "Force_Right_Turn": 4,
            "Force_Left_Turn": 5
        }
        self.state = self.States["Init"]
        # self.FORCED_DRIVE_DONE = False
        self.COUNTER = 0
        self.angle = 0
        self.action_Taken = False
        self.CROSSWALK = False
        self.EMERGENCY = False
        # interface for driving
        self.asyncDrive = asyncDrive()

    def runSupervisorStateMachine(self, laneDetect_routeManagerQ, stopDetect_routeManagerQ, emergencyStop_routeManagerQ):
        self.laneDetectQ = laneDetect_routeManagerQ
        self.stopDetectQ = stopDetect_routeManagerQ
        self.emergencyStopQ = emergencyStop_routeManagerQ

        while True:
            self.angle = self.laneDetectQ.get()
            self.crosswalk = self.stopDetectQ.get()
            self.emergencyStop = self.emergencyStopQ.get()
            self.RouteTick()

    def RouteActions(self):
        # actions to take in each state
        if self.state == self.States["Init"]:
            # do init stuff
            pass
        elif self.state == self.States["Stop"]:
            pass
        elif self.state == self.States["Lane_Follow"]:
            if self.action_Taken == False:
                self.asyncDrive.start_LaneFollowing()
                self.action_Taken = True
            self.asyncDrive.LaneFollow(self.angle)
        elif self.state == self.States["Force_Forward"]:
            if self.action_Taken == False:
                self.asyncDrive.forward()
                self.action_Taken = True
        elif self.state == self.States["Force_Right_Turn"]:
            if self.action_Taken == False:
                self.asyncDrive.right_turn()
                self.action_Taken = True
        elif self.state == self.States["Force_Left_Turn"]:
            if self.action_Taken == False:
                self.asyncDrive.left_turn()
                self.action_Taken = True

    def routePlan(self):
        # returns the next state
        Route = ['Lane_Follow', 'Force_Forward', 'Lane_Follow', 'Force_Right_Turn', 'Lane_Follow', \
                 'Force_Forward', 'Lane_Follow', 'Force_Left_Turn', 'Lane_Follow', 'Force_Right_Turn', \
                 'Lane_Follow', 'Force_Left_Turn', 'Lane_Follow', 'Force_Left_Turn']
        route = Route[self.COUNTER]
        self.COUNTER += 1
        if self.COUNTER > len(Route):
            self.COUNTER = 0

        return route

    def emergencyStop(self):
        return self.EMERGENCY

    def crosswalk(self):
        return self.CROSSWALK

    # def checkForceDrive(self):
    #     return self.asyncDrive.forceDriveDone

    def RouteTick(self):
        # route state transition
        if self.state == self.States["Init"]:
            self.state = self.States["Stop"]

        elif self.state == self.States["Stop"]:
            self.asyncDrive.stop()
            self.state = self.routePlan()

        elif self.state == self.States["Lane_Follow"]:
            if self.emergencyStop():
                self.action_Taken = False
                self.state = self.States["Stop"]
            elif self.crosswalk():
                self.action_Taken = False
                self.state = self.States["Stop"]
            else:
                self.state = self.States["Lane_Follow"]

        elif self.state == self.States["Force_Forward"]:
            if self.emergencyStop():
                self.action_Taken = False
                self.state = self.States["Stop"]
            elif self.asyncDrive.forceDriveDone:
                self.action_Taken = False
                self.asyncDrive.forceDriveDone = False
                self.state = self.States["Stop"]
            else:
                self.state = self.States["Force_Forward"]

        elif self.state == self.States["Force_Right_Turn"]:
            if self.emergencyStop():
                self.action_Taken = False
                self.state = self.States["Stop"]
            elif self.asyncDrive.forceDriveDone:
                self.action_Taken = False
                self.asyncDrive.forceDriveDone = False
                self.state = self.States["Stop"]
            else:
                self.state = self.States["Force_Right_Turn"]

        elif self.state == self.States["Force_Left_Turn"]:
            if self.emergencyStop():
                self.action_Taken = False
                self.state = self.States["Stop"]
            elif self.asyncDrive.forceDriveDone:
                self.action_Taken = False
                self.asyncDrive.forceDriveDone = False
                self.state = self.States["Stop"]
            else:
                self.state = self.States["Force_Left_Turn"]
