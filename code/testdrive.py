#!/usr/bin/python3

import driving

#List of Useful driving values
SPEED_STOP = 0
SPEED_GO = 0.8 
STEER_STRAIGHT = 0
STEER_RIGHT = 25
STEER_LEFT = -25
TEST_DELAY = 1.8
FULL_TURN_DELAY = 1.4
HALF_TURN_DELAY = 1.6
INIT_DELAY = .3

drive = driving.control()

# Preset operations
drive.force_forward_test() # move forward for 2 seconds (test init values)
drive.force_right_turn() # turn right 25, and wait 1.4 seconds with a .3 init time
drive.force_left_turn() # turn left -25, and wait 1.4 seconds with a .3 init time

# Example custom driving operation (wait .3 to start moving, turn 15 degrees for 2s)
drive.force_drive(2,15,0)

