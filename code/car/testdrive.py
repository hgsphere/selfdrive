#!/usr/bin/python3

import driving
import time

#List of Useful driving values
SPEED_STOP = 0
SPEED_GO = 0.8 
STEER_STRAIGHT = 0
STEER_RIGHT = 25
STEER_LEFT = -25
TEST_DELAY = 1.8
FULL_TURN_DELAY = 2
HALF_TURN_DELAY = 1.6
INIT_DELAY = .5

drive = driving.control()

def example():
    # Preset operations
    drive.force_forward_test() # move forward for 2 seconds (test init values)
    drive.force_right_turn() # turn right 25, and wait 1.4 seconds with a .3 init time
    drive.force_left_turn() # turn left -25, and wait 1.4 seconds with a .3 init time

    # Example custom driving operation (wait .3 to start moving, turn 15 degrees for 2s)
    drive.force_drive(2,15,0)

def dounut():
    # Do a Dounut
    # drive.force_drive(2,30)
    # Custum Doununt
    drive.drive(.8)
    drive.drive(.4)
    drive.steer(25)
    time.sleep(3)
    for kk in range(10):
        for i in range(10):
            drive.steer(30)
            drive.steer(15)
        time.sleep(1)
    time.sleep(2)
    drive.force_stop()

def turnloop():
    for kk in range(2):
        for i in range(4):
            drive.force_right_turn()
        for i in range(4):
            drive.force_left_turn()
    drive.force_stop()

# dounut()
turnloop()
# drive.force_right_turn()