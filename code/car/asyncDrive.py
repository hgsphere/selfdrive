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

FPS = 60 # frames per second
SEE_AHEAD = .1 # how far ahead should the car react to

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
        # self.pid = PID(1,0,1.25)
        # self.pid = PID(1,.0075,.75)
        #self.pid = PID(.9,.002,.45)
        #self.pid = PID(.4,.0075,.7) ok with 600 I clip
        #self.pid = PID(.9,.06,.5) great clip I 30
        self.pid = PID(1.5,.003,.8) # .9 .0015 .9 worked previously

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
        self.angles.append(self.bin_angle(angle/4)) # 3 worked well
        self.angle = angle
        self.turn_count = self.turn_count + 1

    def estimate_frame_offest(self):
        speed = self.ctl.get_speed()
        if speed == None or speed < 0  or speed > 2:
            speed = .7

        if speed == 0.0:
            T = 0 
        else:
            T = SEE_AHEAD/speed
        frame_delay = T*FPS
        return round(frame_delay)

    def filter_angles(self):
        # returns filtered angle decision
        # data = np.array(self.angles)
        data = [x if (x <= 30) else 30 for x in list(self.angles)]
        data = [x if (x >= -30) else -30 for x in list(data)]
        #data = self.angles
        #kf = KalmanFilter(initial_state_mean=np.mean(list(data)),initial_state_covariance=np.cov(list(data)),n_dim_obs=1)
        kf = KalmanFilter(initial_state_mean=np.mean(list(data)[0:10]),initial_state_covariance=0.61,n_dim_obs=1)
        means, covs = kf.filter(list(data))
        #estimate = means[14][0]
        self.m = means[0][0] #np.mean([means[i][0] for i in range(9,19)])
        self.c = covs[0][0] #np.mean([covs[i][0] for i in range(9,19)])
        #diff = np.mean(np.diff(means,axis=0))*20
        # print(list(data))
        # print(means)
        #print(covs)
        # print(np.diff(means,axis=0))
        # print(diff)
        # print(list(data)[4:7])
        # print(np.cov(data))
        #frame_delay = self.estimate_frame_offest()
        #print(frame_delay) 
        #if frame_delay == 0:
        #    df = 27
        #elif frame_delay > 27:
        #    df = 2
        #else:
        #    df = 30 - frame_delay - 3

        #if df < 20:
        #    df = 20
        df = 18
        avg = np.mean([means[i][0] for i in range(df-1,df+1)])
        # print("here")
        # print(avg)
        # print("here_after")
        #mm = round(np.mean(list(data)[4:7]),2)
        # print(mm)
        # estimate = estimate if (estimate <=30) else 30
        # estimate = estimate if (estimate >= -30) else -30
        return round(avg, 0)

    def bin_angle(self,angle):
        if angle >= 0:
            return np.floor(angle/2)*2
        else:
            return np.ceil(angle/2)*2

    def LaneFollow(self, angle):
        # updates angle queue changes steering
        self.add_angle(angle)
        
        # test encoder
        #self.ctl.get_encoder()


        f_angle = self.filter_angles()
        #f_angle = self.angle*3/4
        #if f_angle is not self.last_angle:
       
        #self.pid.get(f_angle,True)
        if self.turn_count >= self.turn_skip:
            #if self.initval:
                #self.pid.prev_value = f_angle
                #self.initval = False
            sendme = self.pid.get(f_angle,True,True)
            #sendme = f_angle#round(.75*f_angle)
            self.ctl.steer(self.bin_angle(sendme))
            self.turn_count = 0
            self.last_angle = sendme
