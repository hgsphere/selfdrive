import os
import queue
import sys
import json
import collections
import numpy as np
import time
import cv2 as cv

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
        self.state = self.States["Init"] # The IPS code route planning doesn't return to the lanefollowing
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
        print(os.path.join(globals.code_base_dir, "ips/route.json"))
        with open(os.path.join(globals.code_base_dir, "ips/route.json")) as jf:
            self.route_critical_waypoints = json.load(jf)

        zz = np.zeros((1,30)) 
        self.crossdeque = collections.deque(zz.tolist()[0],maxlen=30)

#    def runSupervisorStateMachine(self, laneDetect_routeManagerQ, stopDetect_routeManagerQ, emergencyStop_routeManagerQ, ips_routeManagerQ):
    def runSupervisorStateMachine(self, laneDetect_routeManagerQ, stopDetect_routeManagerQ, emergencyStop_routeManagerQ, lat, lon):
        self.laneDetectQ = laneDetect_routeManagerQ
        self.stopDetectQ = stopDetect_routeManagerQ
        self.emergencyStopQ = emergencyStop_routeManagerQ
        #self.ipsQ = ips_routeManagerQ
        self.lat = lat
        self.lon = lon

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

            # I don't thinl we can grab these variables this way
            #self.COORDINATES = ips.latitude, ips.longitude
            self.COORDINATES = self.lat.value, self.lon.value
   

            self.RouteTick()
            #print(self.state)

    def RouteActions(self):
        # actions to take in each state
        if self.state == self.States["Init"]:
            self.current_path,name = self.ips.findNextStopLine(self.COORDINATES[0],self.COORDINATES[1])
            print(self.current_path,name)
        elif self.state == self.States["Stop"]:
            self.current_path,name = self.ips.findNextStopLine(self.COORDINATES[0],self.COORDINATES[1])
     
        elif self.state == self.States["Crosswalk_Stop"]:
            pass
        elif self.state == self.States["Lane_Follow"]:
            if self.action_Taken == False:
                self.asyncDrive.start_LaneFollowing()
                print('Starting Lane Following')
                self.action_Taken = True
            #G_angle = self.calc_GPS_angle()
            #print(G_angle) ha I almost had this working but I broke it
            self.asyncDrive.LaneFollow(self.angle)# + 0*G_angle)
        elif self.state == self.States["Force_Forward"]:
            print(self.action_Taken)
            if self.action_Taken == False:
                print("MOVING FOWARD!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                self.asyncDrive.forward()
                self.action_Taken = True
        elif self.state == self.States["Force_Right_Turn"]:
            print(self.action_Taken)
            if self.action_Taken == False:
                print("MOVING RIGHT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                self.asyncDrive.right_turn()
                self.action_Taken = True
        elif self.state == self.States["Force_Left_Turn"]:
            print(self.action_Taken)
            if self.action_Taken == False:
                print("MOVING LEFT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                self.asyncDrive.left_turn()
                self.action_Taken = True

    def routePlan(self):
        # based on the current location, find the next stop line
        #print(self.COORDINATES)
        self.current_path,name = self.ips.findNextStopLine(self.COORDINATES[0],self.COORDINATES[1])
        print(self.current_path)
        # which direction to turn?
        #return ips.computeTurnDirection(self.current_path[0:3])
        #print(self.current_path[0][0:3])
        #print(ips.computeTurnDirection(self.current_path[0][0:3]))
        if (self.current_path is not None) and (len(self.current_path) > 2):
            return self.States[ips.computeTurnDirection(self.current_path[0:3])]
        else: # We might be stuck at a stop here
            pass
            #return self.States["Force_Forward"]

        # if there path couldn't be found got to Init
        return self.States["Init"]

        # returns the next state
        #Route = ['Lane_Follow', 'Force_Forward', 'Lane_Follow', 'Force_Right_Turn', 'Lane_Follow', \
        #         'Force_Forward', 'Lane_Follow', 'Force_Left_Turn', 'Lane_Follow', 'Force_Right_Turn', \
        #         'Lane_Follow', 'Force_Left_Turn', 'Lane_Follow', 'Force_Left_Turn']
        #route = Route[self.COUNTER]
        #self.COUNTER += 1
        #if self.COUNTER > len(Route):
        #    self.COUNTER = 0
        #
        #return self.States[route]

    def inRangeOfCritWaypoint(self):
        """Are we in range of a critical waypoint? These are defined in route.json"""
        w, h, dist = self.ips.findClosestGraphPoint(*self.COORDINATES, getDist=True)
        #self.current_path = self.ips.findNextStopLine(self.COORDINATES[0],self.COORDINATES[1])
        #print(self.current_path)
        #print(self.current_path[self.current_path_idx])
        #print('####################################### ' + str(self.current_path_idx))
        #print(self.COORDINATES)
        # if the node is the close one, and within a distance
        if getPtName(w, h) == self.current_path[self.current_path_idx]: ### PROBLEM HERE current_path_idx is None
            if dist < 20.0:
                # increment the critical waypoint index
                self.current_path_idx += 1
                return  True
        return False

    def inRangeOfStopLine(self):
        """Are we in range of the next stop line? This is the last element in the current path list."""
        w, h, dist = self.ips.findClosestGraphPoint(*self.COORDINATES, getDist=True)
        #self.current_path = self.ips.findNextStopLine(self.COORDINATES[0],self.COORDINATES[1])


        #img = self.ips.displayPath(self.current_path[0])
        #cv.imshow("features", img)
        #time.sleep(5)
        # if the node is the close one, and within a distance
        
        #if getPtName(w, h) == self.current_path[-1]:
        #    if dist < 20.0:
        #        return True

        # This might work
        route,stopname = self.ips.findNextStopLine(self.COORDINATES[0],self.COORDINATES[1]) 
        print(route,stopname)
        print('####################################### ' + str(len(route)))
        print(self.COORDINATES)
        if len(route) == 1:
            return True
        return False

    def emergencyStop(self):
        return self.EMERGENCY

    def crosswalk(self):
        self.crossdeque.append(self.CROSSWALK)

        if np.sum(list(self.crossdeque)[8:22]) > 5:
            print(list(self.crossdeque))
            return True
        return False

    def checkForceDrive(self):
        return self.asyncDrive.forceDriveDone


    def calc_GPS_angle(self):
        """Compute which direction to turn.  Accepts a slice of the path, only 3 nodes needed."""
        nodes,name = self.ips.findNextStopLine(self.COORDINATES[0],self.COORDINATES[1])
        if len(nodes) < 4:
            #print('Less than four waypoints go straight')
            return 0
        #print(nodes[0:4])
        n0,n1,n2,n3 = nodes[0:4]
       
        # get closest waypoint
        w, h, dist = self.ips.findClosestGraphPoint(self.COORDINATES[0],self.COORDINATES[1],getDist=True)
        
        # get the sides of our GPS error trangle
        dist_plan = np.sqrt((n3[0]-n0[0])**2+(n3[1]-n0[1])**2)
        dist_close = dist
        dist_future = np.sqrt((n3[0]-self.COORDINATES[0])**2+(n3[1]-self.COORDINATES[1])**2)
        
        #print(dist_plan,dist_close,dist_future)
        # calculate the angle between the car and perdicted GPS path
        GPS_angle = (180/np.pi)*np.arccos((dist_future**2 + dist_plan**2 - dist_close**2)/(2*abs(dist_plan*dist_future)))

        # slopes      
        if (n1[0] - n0[0]) == 0:
            s0 = 0
        else:
            s0 = (n2[1] - n0[1]) / (n1[0] - n0[0])
        
        if (n2[0] - n1[0]) == 0:                                                       
            s1 = 0
        else:
            s1 = (n3[1] - n0[1]) / (n3[0] - n0[0])
                                                                   
        sdiff = s0 - s1
        
        slopeTolerance = .01
        #print(sdiff)
        #print(s0,s1)                                               
        # make decision
        if abs(sdiff) < slopeTolerance:
            return 0
        elif sdiff > 0:
            return GPS_angle
        elif sdiff < 0:
            return -GPS_angle
        else:
            return 0

    def RouteTick(self):
        print(self.state)
        self.RouteActions()
        # route state transition
        if self.state == self.States["Init"]:
            print('Init State')
            self.state = self.States["Lane_Follow"]

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
###### NOT WORKING
######    So there is nonetype error when checking the current_waypoint state, I gonna comment this out to tune LaneFollowing
            elif self.inRangeOfCritWaypoint():
                self.action_Taken = False
                self.state = self.States["Stop"]

            elif self.inRangeOfStopLine():
                self.action_Taken = False
                self.state = self.States["Stop"]

            else:
                self.state = self.States["Lane_Follow"]

        elif self.state == self.States["Force_Forward"]:
            if self.current_path_idx == len(self.current_path):
                # spin here forever
                self.state = self.States["Stop"]
                self.action_Taken = False

            if self.emergencyStop():
                self.action_Taken = False
                self.state = self.States["Stop"]

            elif self.asyncDrive.forceDriveDone:
                self.action_Taken = False
                self.asyncDrive.forceDriveDone = False
                self.state = self.States["Lane_Follow"]

            else:
                self.state = self.States["Force_Forward"]

        elif self.state == self.States["Force_Right_Turn"]:
            if self.emergencyStop():
                self.action_Taken = False
                self.state = self.States["Stop"]

            elif self.asyncDrive.forceDriveDone:
                self.action_Taken = False
                self.asyncDrive.forceDriveDone = False
                self.state = self.States["Lane_Follow"]

            else:
                self.state = self.States["Force_Right_Turn"]

        elif self.state == self.States["Force_Left_Turn"]:
            if self.emergencyStop():
                self.action_Taken = False
                self.state = self.States["Stop"]

            elif self.asyncDrive.forceDriveDone:
                self.action_Taken = False
                self.asyncDrive.forceDriveDone = False
                self.state = self.States["Lane_Follow"]

            else:
                self.state = self.States["Force_Left_Turn"]
