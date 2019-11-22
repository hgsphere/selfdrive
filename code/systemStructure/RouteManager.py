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
            "GPS_Follow": 8,
            "Force_Forward": 4,
            "Force_Right_Turn": 5,
            "Force_Left_Turn": 6,
            "Crit_wp_stop": 7
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
        self.cornerTurn = False

        self.stopCounter = 0
        self.lat = None
        self.lon = None

        zz = np.zeros((1,30)) 
        self.crossdeque = collections.deque(zz.tolist()[0],maxlen=30)
        self.last_dist_change = 0
        self.last_dist = 0

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

            # I don't thinl we can grab these variables this way
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
                self.asyncDrive.start_LaneFollowing()
                print('Starting GPS Following')
                self.action_Taken = True
            G_angle = self.calc_GPS_angle()
            print(G_angle)
            self.asyncDrive.LaneFollow(self.angle*1+ 0*G_angle)

        elif self.state == self.States["GPS_Follow"]:
            if self.action_Taken == False:
                self.asyncDrive.start_LaneFollowing()
                print('Starting Lane Following')
                self.action_Taken = True
            G_angle = self.calc_GPS_angle()
            print(G_angle) #I almost had this working but I broke it
            self.asyncDrive.LaneFollow(self.angle*0 + -10*G_angle)

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
        # find the path to the next critical waypoint
        # use that data to determine the turn
        self.last_dist = 0
        self.last_dist_change = 0
        print(self.current_path)
        print("We've found a stop line: {}".format(self.name))
        # look-up table for the easy corner stops


        # path from the previous stop line to the next critical waypoint
        # ensure that the first point passed to findPath is the stopLine coordinates
        stopLineCoords = features.getStopLineCoordinates(self.name)
        if stopLineCoords is None:
            stopLineCoords = self.COORDINATES
        else:
            stopLineCoords = decodePtName(stopLineCoords)

        nextTurnPath = self.ips.findPath(*stopLineCoords,
                                         *decodePtName(self.route_critical_waypoints[self.current_path_idx]))
        if self.name is "stopLine0" or self.name is "stopLine3":
            nextTurn = self.States["Force_Left_Turn"]
            self.asyncDrive.corner_turn = True
            nextStart = nextTurnPath[5]
        elif self.name is "stopLine1" or self.name is "stopLine2":
            nextTurn = self.States["Force_Right_Turn"]
            self.asyncDrive.corner_turn = True
            nextStart = nextTurnPath[5]
        elif len(nextTurnPath) >= 3:
            nextTurn = self.States[ips.computeTurnDirection(nextTurnPath[0:5])]
            nextStart = nextTurnPath[5]
        else:
            print("Error! next path is too short!!!!!!!!!!!!")
            nextTurn = self.States["Force_Forward"]
            nextStart = self.COORDINATES


        # now from where the turn ends, find the next stop line
        self.current_path, self.name = self.ips.findNextStopLine(*nextStart)

        # return the next turn
       ## Always do GPS turns
        return self.States["GPS_Follow"] #nextTurn

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
        # w, h, dist = self.ips.findClosestGraphPoint(*self.COORDINATES, getDist=True)
        w, h = self.COORDINATES

        # If all waypoints have been reached restart
        if self.current_path_idx == len(self.route_critical_waypoints):
            self.current_path_idx = 0

        targetPt = decodePtName(self.route_critical_waypoints[self.current_path_idx])
        dist = sqrt(pow(w - targetPt[0], 2) + pow(h - targetPt[1], 2))
        #self.current_path = self.ips.findNextStopLine(self.COORDINATES[0],self.COORDINATES[1])
        #print(self.current_path)
        #print(self.current_path[self.current_path_idx])
        #print('####################################### ' + str(self.current_path_idx))
        #print(self.COORDINATES)
        # if the node is the close one, and within a distance
        # if (w, h) == self.route_critical_waypoints[self.current_path_idx]: ### PROBLEM HERE current_path_idx is None
        if dist < self.ips.avg_dst:
            # increment the critical waypoint index
            print('#######################################\n\t\t CRITICAL WAYPOINT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n#######################################')
            self.current_path_idx += 1
            return  True
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
            path2, name2 = self.ips.findNextStopLine(self.COORDINATES[0],self.COORDINATES[1])
            if self.name is "stopLine5" and name2 is "stopLine3":
                return False
            #self.current_path = path2
            #self.name = name2
            #print(self.current_path,self.name)
        self.last_dist = dist
        self.last_dist_change = dist_change
        #img = self.ips.displayPath(self.current_path[0])
        #cv.imshow("features", img)
        #time.sleep(5)
        # debug
        threshDist = self.ips.avg_dst * 1.6
        #route, stopname = self.ips.findNextStopLine(self.COORDINATES[0], self.COORDINATES[1])
        #Print("{}, {}; {} < {} ?".format(route, stopname, dist, threshDist))
        print("{} < {} ?".format( dist, threshDist))
        print("comparing {} to {}".format((w, h), targetPt))
        #print('####################################### ' + str(len(route)))

        # if the node is the close one, and within a distance
        
        # if (w, h) == targetPt:
        if dist < threshDist:
            print("!!!!!!!!!!!!!!!!!!!!!!! STOP !!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            return True

        # print(self.COORDINATES)
        # if len(route) == 1:
        #     return True
        return False

    def TerminateGPS(self):
        """Are we in range of the next stop line? This is ~ 12th element in the current path list."""
        w, h = self.COORDINATES
        targetPt = self.current_path[12]
        dist = sqrt(pow(w - targetPt[0], 2) + pow(h - targetPt[1], 2))
        
        threshDist = self.ips.avg_dst * 1.6
        
        if dist < threshDist:
            print("!!!!!!!!!!!!!!!!!!!!!!! Switching to LANEFOLLWING !!!!!!!!!!!!!!!!!!!!!!!!!!!!")
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
        #print('Closest point')
        #print(tup)
        #print('Current path')
        #print(self.current_path)
        #indx = np.where(self.current_path == tup)
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


    def heading_diff(self,heading1,heading2):
        # heading1 - heading2 
        #(fix 359 - 1 = 358 to 359 - 1 = -2)
        #(fix 1 - 359 - -358 to 1 - 359 = 2)
        if (heading1 > 270) and (heading2 < 90):
            return (heading1 - 360) - heading2
        
        if (heading1 < 90) and (heading2 > 270):
            return heading1 - (heading2 - 360)

        return heading1 - heading2

    def calc_GPS_angle(self):                                                                                                                                            
        """Compute which direction to turn.  Accepts a slice of the path, only 3 nodes needed."""                                                                        
        
        if (len(self.current_path) < 7):
            return 0

        indx = self.quick_index_lookup()
        nodes = self.current_path[indx:indx+7]                                                                                       
        print(nodes)
        if (len(nodes) < 7):
            return 0

        n0,n1,n2,n3,n4,n5,n6 = nodes[0:7]                                                                                                                                         
        # n0 is closest                                                                                                                                                  
                                                             


        # Compute heading of car to 2 waypoints ahead (n2)
        car_heading = ips.findAbsoluteHeading(self.COORDINATES,n4)     # was n2                                                    

        print('Car to 4 waypoints ahead of closest: {}'.format(car_heading))
        # Compute heading of car to next-next waypoint (n2)                                                                                                              
        # assume car is following the current path's direction                                                                                                           
        car_heading2 = ips.findAbsoluteHeading(self.COORDINATES,n6) # was n3                                                                                                       
        print('Car to 6 waypoints ahead of closest: {}'.format(car_heading2))                                                                                                                                               
        # compare car's heading against the waypoints heading                                                                                                            
        path_heading = ips.findAbsoluteHeading(n0,n4) # was n2                                                                             
        print('Closest Waypoint to two waypoints ahead: {}'.format(path_heading))

        # calulate the curvature of the road                                                                                                                             
        future_heading = ips.findAbsoluteHeading(n0,n6)   # was n3                                                                                
        print('Closest waypoint to 3 waypoints ahead: {}'.format(future_heading) )                                                                       

        # this is the current car heading error                                                                                                                             
        delta_heading = self.heading_diff(car_heading2, car_heading)
        print('car to 4 ahead - car to 6 ahead: {}'.format(delta_heading))                                                                                                                                             
                                                                                                                                                                         
        # calc curvature                                                                                                                                                 
        curvature = self.heading_diff(future_heading,path_heading)
        print('Closest point to 4 ahead - closest to 6 ahead: {}'.format(curvature))

        heading_error = delta_heading - curvature
        print('Heading Error: {}'.format(heading_error))                                                                                                                        
        # add in a little future nudging                                                                                                                                 
        GPS_correction_angle = 1.2*heading_error                                                                                                            
        print(GPS_correction_angle)                                                                                                                                      
        return GPS_correction_angle

    def RouteTick(self):
        #print(self.state)
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
            if self.stopCounter == 60:
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
            if self.stopCounter == 60:
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
                self.state = self.States["Crit_wp_stop"]

            elif self.inRangeOfStopLine():
                self.action_Taken = False
                self.state = self.States["Stop"]

            else:
                self.state = self.States["Lane_Follow"]


        elif self.state == self.States["GPS_Follow"]:
            # print('Lane_Follow')
            if self.emergencyStop():
                self.action_Taken = False
                self.preEmergencyStopState = self.state
                self.state = self.States["Stop"]
            elif self.crosswalk():      # and (self.las
                self.action_Taken = False
                self.state = self.States["Stop"]

            elif self.inRangeOfCritWaypoint():
                self.action_Taken = False
                self.state = self.States["Crit_wp_stop"]

            elif self.TerminateGPS():
                self.action_Taken = False
                self.state = self.States["Stop"]

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

            elif self.asyncDrive.forceDriveDone:
                self.action_Taken = False
                self.asyncDrive.forceDriveDone = False
                self.asyncDrive.corner_turn = False
                self.state = self.States["Lane_Follow"]

            else:
                self.state = self.States["Force_Right_Turn"]

        elif self.state == self.States["Force_Left_Turn"]:
            if self.emergencyStop():
                self.action_Taken = False
                self.preEmergencyStopState = self.state
                self.state = self.States["Stop"]

            elif self.asyncDrive.forceDriveDone:
                self.action_Taken = False
                self.asyncDrive.forceDriveDone = False
                self.asyncDrive.corner_turn = False
                self.state = self.States["Lane_Follow"]

            else:
                self.state = self.States["Force_Left_Turn"]



if __name__ == '__main__':
    print("Not to be run as main!")
