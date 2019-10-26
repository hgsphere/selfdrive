#!/usr/bin/python3

import os
import sys
import collections
import threading
import numpy as np
from pykalman import KalmanFilter

sys.path.append(os.path.abspath("../car/"))
from driving import control
sys.path.append(os.path.abspath("."))
from pid import PID

class asyncDrive:

    def __init__(self):
        self.ctl = control()
        zz = np.zeros((1,20))
        self.angles = collections.deque(zz.tolist()[0],maxlen=20)
        # print(self.angles)
        self.forceDriveDone = False
        self.last_angle = 0
        self.turn_skip = 0
        self.turn_count = 0
        self.angle = 0
        self.m = 0
        self.c = 0
        self.initval = True
        #self.pid = PID(1,0,1.25)
        #self.pid = PID(1,.0075,.75)
        self.pid = PID(.9,.002,.45)

    def start_LaneFollowing(self,):
        self.ctl.drive(self.ctl.SPEED_GO)
        self.ctl.drive(self.ctl.SPEED_SLOW)

    def right_turn(self):
        if self.forceDriveDone:
            right_thread = threading.Thread(target=self.async_right_turn())
            right_thread.start()

    def left_turn(self):
        if self.forceDriveDone:
            left_thread = threading.Thread(target=self.async_left_turn())
            left_thread.start()

    def forward(self):
        if self.forceDriveDone:
            forward_thread = threading.Thread(target=self.async_forward())
            forward_thread.start()

    def async_right_turn(self):
        self.forceDriveDone = False
        self.ctl.force_right_turn()

    def async_left_turn(self):
        self.forceDriveDone = False
        self.ctl.force_left_turn()
        self.forceDriveDone = True

    def async_forward(self):
        self.forceDriveDone = False
        self.ctl.force_forward_test()
        self.forceDriveDone = True

    def stop(self):
        self.ctl.force_stop()

    def add_angle(self, angle):
        self.angles.append(angle)
        self.angle = angle
        self.turn_count = self.turn_count + 1

    def filter_angles(self):
        # returns filtered angle decision
        # data = np.array(self.angles)
        data = [x if (x <= 30) else 30 for x in list(self.angles)]
        data = [x if (x >= -30) else -30 for x in list(data)]
        #data = self.angles
        #kf = KalmanFilter(initial_state_mean=np.mean(list(data)),initial_state_covariance=np.cov(list(data)),n_dim_obs=1)
        kf = KalmanFilter(initial_state_mean=np.mean(list(data)),initial_state_covariance=0.6,n_dim_obs=1)
        means, covs = kf.filter(list(data))
        estimate = means[14][0]
        self.m = means[0][0] #np.mean([means[i][0] for i in range(9,19)])
        self.c = covs[0][0] #np.mean([covs[i][0] for i in range(9,19)])
        #diff = np.mean(np.diff(means,axis=0))*20
        # print(list(data))
        # print(means)
        # print(covs)
        # print(np.diff(means,axis=0))
        # print(diff)
        # print(list(data)[4:7])
        # print(np.cov(data))
        avg = np.mean([means[i][0] for i in range(15,16)])
        # print("here")
        # print(avg)
        # print("here_after")
        #mm = round(np.mean(list(data)[4:7]),2)
        # print(mm)
        # estimate = estimate if (estimate <=30) else 30
        # estimate = estimate if (estimate >= -30) else -30
        return round(avg, 0)

    def LaneFollow(self, angle):
        # updates angle queue changes steering
        self.add_angle(angle)

        f_angle = self.filter_angles()
        #f_angle = self.angle*3/4
        #if f_angle is not self.last_angle:
       
        #self.pid.get(f_angle,True)
        if self.turn_count >= self.turn_skip:
            if self.initval:
                self.pid.prev_value = f_angle
                self.initval = False
            #sendme = self.pid.get(f_angle,False,True)
            sendme = round(.75*f_angle)
            self.ctl.steer(sendme)
            self.turn_count = 0
            self.last_angle = sendme
