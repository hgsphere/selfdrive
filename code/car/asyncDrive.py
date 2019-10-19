#!/usr/bin/python3

import os
import sys
import collections
import threading
import numpy as np

sys.path.append(os.path.abspath("../car/"))
from driving import control


class asyncDrive:

    def __init__(self):
        self.ctl = control()
        self.angles = collections.deque(maxlen=15)
        self.forceDriveDone = False

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

    def filter_angles(self):
        # returns filtered angle decision
        # data = np.array(self.angles)
        data = [x if (x <= 30) else 30 for x in list(self.angles)]
        data = [x if (x >= -30) else -30 for x in list(data)]

        return np.mean(data)

    def LaneFollow(self, angle):
        # updates angle queue changes steering
        self.add_angle(angle)

        f_angle = self.filter_angles()
        self.ctl.steer(f_angle)