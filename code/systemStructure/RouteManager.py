
class RouteManager():
    def runSupervisorStateMachine(self, laneDetect_routeManagerQ, stopDetect_routeManagerQ, emergencyStop_routeManagerQ):
        while True:
            laneDetect_routeManagerQ.get()
            stopDetect_routeManagerQ.get()
            emergencyStop_routeManagerQ.get()