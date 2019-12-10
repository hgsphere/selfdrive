
import os
import queue
import sys
import json
import collections
import numpy as np
import time
import cv2 as cv
from math import sqrt
import features

sys.path.append(os.path.abspath("../car"))
sys.path.append(os.path.abspath("../ips"))
from asyncDrive import asyncDrive
from digraph import getPtName, decodePtName
import ips
import globals


class RouteManager(object):

    def __init__(self):
        self.laneDetectQ = None
        self.stopDetectQ = None
        self.emergencyStopQ = None
        self.ipsQ = None
        self.name = None

        self.States = {
            "Init": 0,
            "Stop": 1,
            "Crosswalk_Stop": 2,
            "Lane_Follow": 3,
            "Force_Forward": 4,
            "Force_Right_Turn": 5,
            "Force_Left_Turn": 6,
            "Crit_wp_stop": 7,
            "GPS_Follow": 8,
            "Wait_for_green": 9
        }
        self.state = self.States["Init"] # The IPS code route planning doesn't return to the lanefollowing
        self.preEmergencyStopState = None
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
        self.nearStopThreshold = self.ips.avg_dst * 4
        self.corner_turn = False
        self.threshDist = self.ips.avg_dst * 7

        # index in current_path for where start turning the wheel, and go back to lane following
        self.turn_pt = None
        self.crossover_pt = None

        # stop at each stop line for about 1 second
        self.stopCounter = 0

        # shared globals
        self.lat = None
        self.lon = None
        self.greenFlag = None
        self.stop_now_flag = None
        self.reset_yolo_flag = None

        # history variables
        zz = np.zeros((1,30)) 
        self.crossdeque = collections.deque(zz.tolist()[0],maxlen=30)
        zz = np.zeros((10,2))
        self.coords_hist = collections.deque(zz.tolist(),maxlen=10)
        zz = np.zeros((1,10))
        self.heading_hist = collections.deque(zz.tolist()[0],maxlen=10)
        self.last_coords = (0,0)
        self.last_dist_change = 0
        self.last_dist = 0
        self.nextTurn = None
        self.prev_GPS_angle = 0

    def runSupervisorStateMachine(self, laneDetect_routeManagerQ, stopDetect_routeManagerQ, emergencyStop_routeManagerQ,
                                  lat, lon, yolo_green_flag, stop_now_flag, reset_yolo_flag):
        self.laneDetectQ = laneDetect_routeManagerQ
        self.stopDetectQ = stopDetect_routeManagerQ
        self.emergencyStopQ = emergencyStop_routeManagerQ
        #self.ipsQ = ips_routeManagerQ
        self.lat = lat
        self.lon = lon
        self.greenFlag = yolo_green_flag
        self.stop_now_flag = stop_now_flag
        self.reset_yolo_flag = reset_yolo_flag

        while True:
            self.angle = self.laneDetectQ.get()

            #print(self.angle, end='\t')
            self.CROSSWALK = self.stopDetectQ.get()

            #print(self.CROSSWALK, end='\t')
            self.EMERGENCY = self.emergencyStopQ.get(timeout=1000)

            #print(self.EMERGENCY, end='\t')

            # try:
            #     coords = self.ipsQ.get_nowait()
            #     self.COORDINATES = coords
            # except queue.Empty as e:
            #     pass

            # I don't think we can grab these variables this way
            #self.COORDINATES = ips.latitude, ips.longitude
            self.COORDINATES = abs(self.lat.value), abs(self.lon.value)

            self.RouteTick()
            #print(self.state)

    def RouteActions(self):
        # actions to take in each state
        if self.state == self.States["Init"]:
            self.current_path, self.name = self.ips.findNextStopLine(self.COORDINATES[0],self.COORDINATES[1])
            print(self.current_path,self.name)
        elif self.state == self.States["Stop"]:
            # self.current_path, name = self.ips.findNextStopLine(self.COORDINATES[0],self.COORDINATES[1])
            pass

        elif self.state == self.States["Crosswalk_Stop"]:
            pass

        elif self.state == self.States["Lane_Follow"]:
            if self.action_Taken == False:
                # self.asyncDrive.clear()
                self.asyncDrive.start_LaneFollowing()
                self.asyncDrive.setPID(.9,.0015,.9)
                print('Starting GPS Following')
                self.action_Taken = True
            G_angle = self.calc_GPS_angle()
            print(G_angle)
            # TODO: mix GPS into this
            self.asyncDrive.LaneFollow(self.angle*1*0.85 + 0.15*3*G_angle)
            # self.asyncDrive.LaneFollow(self.angle * 1)

        elif self.state == self.States["GPS_Follow"]:
            if self.action_Taken == False:
                self.asyncDrive.clear()
                self.asyncDrive.start_LaneFollowing()
                # self.asyncDrive.setPID(.9,.0015,.9)
                self.asyncDrive.setPID(.9, .0015, 0.9)
                print('Starting Lane Following')
                self.action_Taken = True
            G_angle = self.calc_GPS_angle()
            G_angle = self.asyncDrive.pid.thres(G_angle, 15)
            print(G_angle)
            self.asyncDrive.LaneFollow(self.angle*0 + 2.0*G_angle)

        elif self.state == self.States["Force_Forward"]:
            print(self.action_Taken)
            if self.action_Taken == False:
                print("MOVING FOWARD!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                self.asyncDrive.forward()
                self.action_Taken = True
        elif self.state == self.States["Force_Right_Turn"]:
            print(self.action_Taken)
            # if self.action_Taken == False:
            #     print("MOVING RIGHT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            #     self.asyncDrive.right_turn()
            #     self.action_Taken = True
            if self.action_Taken == False:
                self.asyncDrive.clear()
                self.asyncDrive.start_LaneFollowing()
                self.asyncDrive.setPID(.9,.0015,.9)
                print('Starting Right Turn')
                self.action_Taken = True

            # detect if we've hit the turning point
            if self.inRangeOfGenericPt(self.turn_pt):
                self.asyncDrive.steer_right()

        elif self.state == self.States["Force_Left_Turn"]:
            print(self.action_Taken)
            # if self.action_Taken == False:
            #     print("MOVING LEFT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            #     self.asyncDrive.left_turn()
            #     self.action_Taken = True
            if self.action_Taken == False:
                self.asyncDrive.clear()
                self.asyncDrive.start_LaneFollowing()
                self.asyncDrive.setPID(.9,.0015,.9)
                print('Starting Left Turn')
                self.action_Taken = True

            if self.inRangeOfGenericPt(self.turn_pt):
                self.asyncDrive.steer_left()

        elif self.state == self.States["Wait_for_green"]:
            pass

    def routePlan(self):
        # based on the current location, find the next stop line
        #print(self.COORDINATES)
        # find the path to the next critical waypoint
        # use that data to determine the turn
        self.last_dist = 0
        self.last_dist_change = 0
        print(self.current_path)
        print("We've found a stop line: {}".format(self.name))
        old_name = self.name
        # look-up table for the easy corner stops

        # path from the previous stop line to the next critical waypoint
        # ensure that the first point passed to findPath is the stopLine coordinates
        stopLineCoords = features.getStopLineCoordinates(self.name)
        if stopLineCoords is None:
            stopLineCoords = self.COORDINATES
        else:
            stopLineCoords = decodePtName(stopLineCoords)

        # now plan a path from the stop line to the next critical waypoint
        nextTurnPath = self.ips.findPath(*stopLineCoords,
                                         *decodePtName(self.route_critical_waypoints[self.current_path_idx]))

        # determine which direction we're supposed to turn
        if self.name is "stopLine0" or self.name is "stopLine3":
            nextTurn = self.States["Force_Left_Turn"]
            self.corner_turn = True
            nextStart = nextTurnPath[5]
        elif self.name is "stopLine1" or self.name is "stopLine2":
            nextTurn = self.States["Force_Right_Turn"]
            self.corner_turn = True
            nextStart = nextTurnPath[5]
        elif len(nextTurnPath) >= 3:
            self.corner_turn = False
            computedDirection = ips.computeTurnDirection(nextTurnPath[0:5])
            nextTurn = self.States[computedDirection]
            nextStart = nextTurnPath[5]
        else:
            self.corner_turn = False
            print("Error! next path is too short!!!!!!!!!!!!")
            nextTurn = self.States["Force_Forward"]
            nextStart = self.COORDINATES

        # now from where the turn ends, find the next stop line
        print("Current coords are {}".format(self.COORDINATES))
        print("Starting next path calculation from {}".format(nextStart))
        self.current_path, self.name = self.ips.findNextStopLine(*nextStart)
        print("Now we're going to stop line: {}".format(self.name))

        # figure out the point where the GPS turn is supposed to start and stop
        print("Going from {} to {} turning {}".format(old_name, self.name, nextTurn))
        turnStartPt = self.ips.getTurningPt(old_name, nextTurn)
        # crossoverPt = self.ips.getCrossoverPt(self.name)
        straightPt = self.ips.getStraightPt(self.name)
        # print(" - The crossover point is {} - ".format(crossoverPt))
        # recalculate the full path from current location to the next stop line
        self.current_path = self.ips.findPath(*self.COORDINATES, *self.current_path[-1])

        # find the index in the full path for when to start turning
        self.turn_pt = decodePtName(turnStartPt)
        # find the index in the full path for when to stop turning
        self.crossover_pt = decodePtName(straightPt)

        # return the next turn
        ## Always do GPS turns
        if self.corner_turn:
            self.asyncDrive.ctl.corner_turn = True
            return nextTurn #self.States["GPS_Follow"]  # nextTurn
        # if the light is green, go ahead and skip waiting
        else:
            self.asyncDrive.ctl.corner_turn = False
            self.nextTurn = nextTurn
            # reset box boundaries at the traffic light
            self.reset_yolo_flag.value = 1
            return self.States["Wait_for_green"]

    # print(self.current_path)
        # which direction to turn?
        #return ips.computeTurnDirection(self.current_path[0:5])
        #print(self.current_path[0][0:3])
        #print(ips.computeTurnDirection(self.current_path[0][0:5]))
        # if (self.current_path is not None) and (len(self.current_path) > 2):
        #     return self.States[ips.computeTurnDirection(self.current_path[0:5])]
        # else: # We might be stuck at a stop here
        #     pass
            #return self.States["Force_Forward"]

        # if there path couldn't be found got to Init
        # return self.States["Init"]

    def inRangeOfGenericPt(self, targetPt, threshold=80):
        w, h = self.COORDINATES
        dist = sqrt(pow(w - targetPt[0], 2) + pow(h - targetPt[1], 2))

        if dist < threshold:
            print("{} is within {} of {}".format(self.COORDINATES, dist, targetPt))
            return  True
        else:
            return False

    def inRangeOfCritWaypoint(self):
        """Are we in range of a critical waypoint? These are defined in route.json"""
        # w, h = self.COORDINATES

        # If all waypoints have been reached restart
        if self.current_path_idx == len(self.route_critical_waypoints):
            self.current_path_idx = 0

        targetPt = decodePtName(self.route_critical_waypoints[self.current_path_idx])
        # dist = sqrt(pow(w - targetPt[0], 2) + pow(h - targetPt[1], 2))
        # if the node is the close one, and within a distance

        if self.inRangeOfGenericPt(targetPt, threshold=self.ips.avg_dst * 2):
            print('#######################################\n\t\t CRITICAL WAYPOINT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n#######################################')
            # increment the critical waypoint index
            self.current_path_idx += 1
            return  True
        else:
            return False

    def inRangeOfStopLine(self):
        """Are we in range of the next stop line? This is the last element in the current path list."""
        # w, h, dist = self.ips.findClosestGraphPoint(*self.COORDINATES, getDist=True)
        w, h = self.COORDINATES
        targetPt = self.current_path[-1]
        dist = sqrt(pow(w - targetPt[0], 2) + pow(h - targetPt[1], 2))
        #self.current_path = self.ips.findNextStopLine(self.COORDINATES[0],self.COORDINATES[1])
        
        # check if the car is not following the current path
        dist_change = self.last_dist - dist
        #wrong_direction_thres = 1
        if (dist_change < 0) and (dist_change < self.last_dist_change):
            # Recalculate the current path
            pass
            #path2, name2 = self.ips.findNextStopLine(self.COORDINATES[0],self.COORDINATES[1])
            #if self.name is "stopLine5" and name2 is "stopLine3":
            #    return False
            #self.current_path = path2
            #self.name = name2
            #print(self.current_path,self.name)
        self.last_dist = dist
        self.last_dist_change = dist_change
        #img = self.ips.displayPath(self.current_path[0])
        #cv.imshow("features", img)
        #time.sleep(5)
        # debug

        #route, stopname = self.ips.findNextStopLine(self.COORDINATES[0], self.COORDINATES[1])
        #Print("{}, {}; {} < {} ?".format(route, stopname, dist, threshDist))
        print("{} < {} ?".format( dist, self.threshDist))
        print("comparing {} to {}".format((w, h), targetPt))
        #print('####################################### ' + str(len(route)))

        # if the node is the close one, and within a distance
        
        # if (w, h) == targetPt:
        if dist < self.threshDist:
            print("!!!!!!!!!!!!!!!!!!!!!!! STOP !!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            return True

        # print(self.COORDINATES)
        # if len(route) == 1:
        #     return True
        return False

    def TerminateGPS(self):
        """Are we in range of the next stop line? This is ~ 12th element in the current path list."""
        w, h = self.COORDINATES
        targetPt = self.crossover_pt
        dist = sqrt(pow(w - targetPt[0], 2) + pow(h - targetPt[1], 2))
        
        threshDist = self.ips.avg_dst * 3
        print("comparing {} to {}".format((w, h), targetPt))
        
        if dist < threshDist:
            print("\n\n!!!!!!!!!!!!!!!!!!!!!!! Switching to LANEFOLLWING !!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n")
            return True

        return False

    def emergencyStop(self):
        return self.EMERGENCY

    def crosswalk(self):
        self.crossdeque.append(self.CROSSWALK)

        if np.sum(list(self.crossdeque)[19:29]) > 8:
            print(list(self.crossdeque))
            return False
        return False

    def checkForceDrive(self):
        return self.asyncDrive.forceDriveDone

    def quick_index_lookup(self):
        w, h = self.ips.findClosestPathPoint(self.current_path,*self.COORDINATES, getDist=False)
        tup = (w,h)
        try:
            indx = self.current_path.index(tup)
        except Exception as e:
            print(tup)
            print(self.current_path)
            indx = 0
        #print(indx)
        if indx is None:
            print('Could Not find Car')
            return 0
        #print(indx)
        return indx

    def gen_arc(self, head, radius, coordinates):
        arcCount = 20
        pheads = np.linspace(head - 60, head + 60, num=arcCount)
        # print(pheads)
        x = np.cos(np.deg2rad(pheads)) * radius + coordinates[0]
        y = np.sin(np.deg2rad(pheads)) * radius + coordinates[1]
        pcoords = np.zeros((arcCount, 2))
        for i in range(0, arcCount):
            pcoords[i, :] = x[i], y[i]
        return pcoords, pheads



    def heading_diff(self, heading1, heading2):
        # heading1 - heading2 
        #(fix 359 - 1 = 358 to 359 - 1 = -2)
        #(fix 1 - 359 - -358 to 1 - 359 = 2)
        if (heading1 > 270) and (heading2 < 90):
            return (heading1 - 360) - heading2 
        
        if (heading1 < 90) and (heading2 > 270):
            return heading1 - (heading2 - 360)

        return heading1 - heading2

    def find_look_ahead(self, head, rad):
        head = head - 90
        pcoords, pheads = self.gen_arc(head, rad, self.COORDINATES)
        indx = self.quick_index_lookup()
        pathSlice = self.current_path[indx:]
        distArray = np.zeros((len(pcoords), len(pathSlice)))

        for i, arcpt in enumerate(pcoords, start=0):
            for j, pt in enumerate(pathSlice):
                dist = sqrt(pow(arcpt[0] - pt[0], 2) + pow(arcpt[1] - pt[1], 2))
                distArray[i][j] = dist

        minIdx = np.unravel_index(np.argmin(distArray, axis=None), distArray.shape)
        # print(pcoords)
        # print("Minimum index is {}".format(minIdx))
        minPt = pheads[minIdx[0]]
        print("Minimum point is {}".format(minPt))
        return self.heading_diff(minPt, head)

    def calc_GPS_angle(self):
        """Compute which direction to turn.  Accepts a slice of the path, only 3 nodes needed."""

        if (len(self.current_path) < 3):
            return 0

        indx = self.quick_index_lookup()
        nodes = self.current_path[indx:indx + 3]
        # print(nodes)
        if (len(nodes) < 3):
            return 0

        n0, n1, n2 = nodes[0:3]
        # n0 is closest

        if self.last_coords == self.COORDINATES:
            return 0

        #### New method (heading change based)
        # compute gps based heading of car
        head = ips.findAbsoluteHeading(self.last_coords, self.COORDINATES)
        lasthead = list(self.heading_hist)[9]
        self.heading_hist.append(head)
        # avgHead = np.mean(list(self.heading_hist)[8:9]) # avg last 3 headings (deal with errornous GPS)
        avgHead = lasthead
        # find the heading the car should be at
        nexthead = ips.findAbsoluteHeading(self.COORDINATES, n2)
        # dist = sqrt(pow(self.COORDINATES[0] - n1[0], 2) + pow(self.COORDINATES[1] - n1[1], 2))

        heading_offset = self.heading_diff(nexthead, avgHead)

        self.last_coords = self.COORDINATES
        print(heading_offset)
        return heading_offset

    def calc_GPS_angle_new(self):
        """Compute which direction to turn.  Accepts a slice of the path, only 3 nodes needed."""
        lookahead_num = 3

        if len(self.current_path) < lookahead_num:
            return 0

        indx = self.quick_index_lookup()
        nodes = self.current_path[indx:indx + lookahead_num]
        # print(nodes)
        if len(nodes) < lookahead_num:
            return 0

        # n0, n1, n2 = nodes[0:3]
        n2 = nodes[lookahead_num-1]
        # n0 is closest

        if self.last_coords == self.COORDINATES:
            return self.prev_GPS_angle

        self.coords_hist.append(self.COORDINATES)
        old_coords = list(self.coords_hist)[7]

        #### New method (heading change based)
        # compute gps based heading of car
        if old_coords == (0, 0):
            head = ips.findAbsoluteHeading(self.last_coords, self.COORDINATES)
        else:
            head = ips.findAbsoluteHeading(old_coords, self.COORDINATES)
        lasthead = list(self.heading_hist)[9]
        self.heading_hist.append(head)
        # avgHead = np.mean(list(self.heading_hist)[8:9]) # avg last 3 headings (deal with errornous GPS)
        avgHead = lasthead

        # heading_offset = self.find_look_ahead(avgHead, 100)
        # find the heading the car should be at
        nexthead = ips.findAbsoluteHeading(self.COORDINATES, n2)
        # #dist = sqrt(pow(self.COORDINATES[0] - n1[0], 2) + pow(self.COORDINATES[1] - n1[1], 2))
        #
        heading_offset = self.heading_diff(nexthead, avgHead)
        # print(list(self.heading_hist))
        self.last_coords = self.COORDINATES
        self.prev_GPS_angle = heading_offset
        return heading_offset

    def RouteTick(self):
        print('State Num: {}'.format(self.state))
        self.RouteActions()
        # route state transition
        if self.state == self.States["Init"]:
            print('Init State')
            self.state = self.States["Lane_Follow"]

        elif self.state == self.States["Stop"]:
            print('Stop State')
            if self.preEmergencyStopState is not None:
                if not self.emergencyStop():
                    self.state = self.preEmergencyStopState
                    self.preEmergencyStopState = None
                    return
                    # self.asyncDrive.start_LaneFollowing()
                else:
                    self.asyncDrive.stop()
                    print("Waiting for object to be removed")
                    return

            if self.stopCounter == 0:
                self.asyncDrive.stop()
            self.stopCounter += 1
            if self.stopCounter == 30:
                self.stopCounter = 0
                self.state = self.routePlan()

        elif self.state == self.States["Crosswalk_Stop"]:
            # if self.crosswalk_y > 400:
            #    self.state = self.States["Stop"]
            self.state = self.States["Stop"]

        elif self.state == self.States["Crit_wp_stop"]:
            if self.stopCounter == 0:
                self.asyncDrive.stop()
            self.stopCounter += 1
            if self.stopCounter == 20:
                self.stopCounter = 0
                self.state = self.States["Lane_Follow"]

        elif self.state == self.States["Lane_Follow"]:
            # print('Lane_Follow')
            if self.emergencyStop():
                self.action_Taken = False
                self.preEmergencyStopState = self.state
                self.state = self.States["Stop"]
            elif self.crosswalk():      # and (self.last_dist < self.nearStopThreshold):
                self.action_Taken = False
                self.state = self.States["Stop"]

            elif self.inRangeOfCritWaypoint():
                self.action_Taken = False
            #     self.state = self.States["Crit_wp_stop"]

            elif self.inRangeOfStopLine():
                self.action_Taken = False
                self.state = self.States["Stop"]

            # stop if YOLO detected that we're close to the traffic light
            elif self.stop_now_flag.value > 0:
                self.action_Taken = False
                self.state = self.States["Stop"]

            else:
                self.state = self.States["Lane_Follow"]

            # debugging YOLO detector
            print("\n\n==========================================================")
            # print("Yolo detected green: {}".format(self.greenFlag.value))
            # print("\n\n")
            # end YOLO section

        elif self.state == self.States["GPS_Follow"]:
            # print('Lane_Follow')
            if self.emergencyStop():
                self.action_Taken = False
                self.preEmergencyStopState = self.state
                self.state = self.States["Stop"]
            elif self.crosswalk():      # and (self.las
                self.action_Taken = False
                self.state = self.States["Stop"]

            # elif self.inRangeOfCritWaypoint():
            #     self.action_Taken = False
            #     self.state = self.States["Crit_wp_stop"]

            elif self.TerminateGPS():
                self.action_Taken = False
                self.state = self.States["Lane_Follow"]

            else:
                self.state = self.States["GPS_Follow"]

        elif self.state == self.States["Force_Forward"]:
            if self.current_path_idx == len(self.current_path):
                # spin here forever
                self.state = self.States["Lane_Follow"]
                self.action_Taken = False

            if self.emergencyStop():
                self.action_Taken = False
                self.preEmergencyStopState = self.state
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
                self.preEmergencyStopState = self.state
                self.state = self.States["Stop"]

            elif self.inRangeOfGenericPt(self.crossover_pt):
                self.action_Taken = False
                self.state = self.States["Lane_Follow"]

            # elif self.asyncDrive.forceDriveDone:
            #     self.action_Taken = False
            #     self.asyncDrive.forceDriveDone = False
            #     self.corner_turn = False
            #     self.state = self.States["Lane_Follow"]

            else:
                self.state = self.States["Force_Right_Turn"]

        elif self.state == self.States["Force_Left_Turn"]:
            if self.emergencyStop():
                self.action_Taken = False
                self.preEmergencyStopState = self.state
                self.state = self.States["Stop"]

            elif self.inRangeOfGenericPt(self.crossover_pt):
                self.action_Taken = False
                self.state = self.States["Lane_Follow"]

            # elif self.asyncDrive.forceDriveDone:
            #     self.action_Taken = False
            #     self.asyncDrive.forceDriveDone = False
            #     self.corner_turn = False
            #     self.state = self.States["Lane_Follow"]

            else:
                self.state = self.States["Force_Left_Turn"]

        elif self.state == self.States["Wait_for_green"]:
            if self.greenFlag.value:
                if self.nextTurn == self.States["Force_Forward"]:
                    self.state = self.States["GPS_Follow"]
                else:
                    self.state = self.nextTurn

                #quiting GPS turns for now
                #self.state = self.States["GPS_Follow"]
            # debugging YOLO detector                                                                                                                                    
            print("\n\n==========================================================")                                                                                      
            print("Yolo detected green: {}".format(self.greenFlag.value))                                                                                                
            print("\n\n")                                                                                                                                                
            # end YOLO section


if __name__ == '__main__':
    print("Not to be run as main!")
