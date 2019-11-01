import os
import queue
import sys
import json
import collections
import numpy as np

sys.path.append(os.path.abspath("../car"))
sys.path.append(os.path.abspath("../ips"))
from asyncDrive import asyncDrive
from digraph import getPtName
import ips
import globals


class RouteManager(object):

    def __init__(self):
        self.laneDetectQ = None
        self.stopDetectQ = None
        self.emergencyStopQ = None
        self.ipsQ = None

        self.States = {
            "Init": 0,
            "Stop": 1,
            "Crosswalk_Stop": 2,
            "Lane_Follow": 3,
            "Force_Forward": 4,
            "Force_Right_Turn": 5,
            "Force_Left_Turn": 6
        }
        self.state = self.States["Init"]
        # self.FORCED_DRIVE_DONE = False
        self.COUNTER = 0
        self.angle = 0
        self.action_Taken = False
        self.CROSSWALK = False
        self.EMERGENCY = False
        self.COORDINATES = None
        # interface for driving
        self.asyncDrive = asyncDrive()
        # route planning along the global path
        self.ips = ips.IPS()
        self.current_path = None
        self.current_path_idx = 0
        with open(os.path.join(globals.code_base_dir, "ips/route.json")) as jf:
            self.route_critical_waypoints = json.load(jf)

        zz = np.zeros((1,30)) 
        self.crossdeque = collections.deque(zz.tolist()[0],maxlen=30)

    def runSupervisorStateMachine(self, laneDetect_routeManagerQ, stopDetect_routeManagerQ, emergencyStop_routeManagerQ, ips_routeManagerQ):
        self.laneDetectQ = laneDetect_routeManagerQ
        self.stopDetectQ = stopDetect_routeManagerQ
        self.emergencyStopQ = emergencyStop_routeManagerQ
        self.ipsQ = ips_routeManagerQ

        while True:
            self.angle = self.laneDetectQ.get()

            print(self.angle, end='\t')
            self.CROSSWALK = self.stopDetectQ.get()

            print(self.CROSSWALK, end='\t')
            self.EMERGENCY = self.emergencyStopQ.get()

            print(self.EMERGENCY, end='\t')

            # try:
            #     coords = self.ipsQ.get_nowait()
            #     self.COORDINATES = coords
            # except queue.Empty as e:
            #     pass
            self.COORDINATES = ips.latitude, ips.longitude

            self.RouteTick()
            # print(self.state)

    def RouteActions(self):
        # actions to take in each state
        if self.state == self.States["Init"]:
            # do init stuff
            pass
        elif self.state == self.States["Stop"]:
            pass
        elif self.state == self.States["Crosswalk_Stop"]:
            pass
        elif self.state == self.States["Lane_Follow"]:
            if self.action_Taken == False:
                self.asyncDrive.start_LaneFollowing()
                # print('Starting Lane Following')
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
        # based on the current location, find the next stop line
        self.current_path = self.ips.findNextStopLine(self.COORDINATES)
        # which direction to turn?
        return ips.computeTurnDirection(self.current_path[0:3])

        # returns the next state
        # Route = ['Lane_Follow', 'Force_Forward', 'Lane_Follow', 'Force_Right_Turn', 'Lane_Follow', \
        #          'Force_Forward', 'Lane_Follow', 'Force_Left_Turn', 'Lane_Follow', 'Force_Right_Turn', \
        #          'Lane_Follow', 'Force_Left_Turn', 'Lane_Follow', 'Force_Left_Turn']
        # route = Route[self.COUNTER]
        # self.COUNTER += 1
        # if self.COUNTER > len(Route):
        #     self.COUNTER = 0
        #
        # return self.States[route]

    def inRangeOfCritWaypoint(self):
        """Are we in range of a critical waypoint? These are defined in route.json"""
        w, h, dist = self.ips.findClosestGraphPoint(*self.COORDINATES, getDist=True)
        # if the node is the close one, and within a distance
        if getPtName(w, h) == self.current_path[self.current_path_idx]:
            if dist < 20.0:
                # increment the critical waypoint index
                self.current_path_idx += 1
                return  True
        return False

    def inRangeOfStopLine(self):
        """Are we in range of the next stop line? This is the last element in the current path list."""
        w, h, dist = self.ips.findClosestGraphPoint(*self.COORDINATES, getDist=True)
        # if the node is the close one, and within a distance
        if getPtName(w, h) == self.current_path[-1]:
            if dist < 20.0:
                return True
        return False

    def emergencyStop(self):
        return self.EMERGENCY

    def crosswalk(self):
        self.crossdeque.append(self.CROSSWALK)

        if np.sum(list(self.crossdeque)[8:22]) > 8:
            print(list(self.crossdeque))
            return True
        return False

    # def checkForceDrive(self):
    #     return self.asyncDrive.forceDriveDone

    def RouteTick(self):
        self.RouteActions()
        # route state transition
        if self.state == self.States["Init"]:
            print('Init State')
            self.state = self.States["Stop"]

        elif self.state == self.States["Stop"]:
            print('Stop State')
            self.asyncDrive.stop()
            self.state = self.routePlan()

        elif self.state == self.States["Crosswalk_Stop"]:
            # if self.crosswalk_y > 400:
            #    self.state = self.States["Stop"]
            self.state = self.States["Stop"]

        elif self.state == self.States["Lane_Follow"]:
            # print('Lane_Follow')
            if self.EMERGENCY:
                self.action_Taken = False
                self.state = self.States["Stop"]
            elif self.crosswalk():
                self.action_Taken = False
                self.state = self.States["Crosswalk_Stop"]

            # things that additionally will be checked for stop:
            #  in range of a critical waypoint
            #  in range of the stop line
            elif self.inRangeOfCritWaypoint():
                self.state = self.States["Stop"]
            elif self.inRangeOfStopLine():
                self.state = self.States["Stop"]

            else:
                self.state = self.States["Lane_Follow"]

        elif self.state == self.States["Force_Forward"]:
            if self.current_path_idx == len(self.current_path):
                # spin here forever
                self.state = self.States["Stop"]
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
