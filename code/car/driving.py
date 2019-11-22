#!/usr/bin/python3

import serial
import time
import datetime

# Assumed Arduino Serial device
ARDUINO_SERIAL = "/dev/ttyUSB0"

# Setting default command history file
HISTORY_FILE = 'command_history.txt'

# List of Default calibration values
DEFAULT_STRAIGHT = 1560
DEFUALT_START = 1660#1685
DEFUALT_SPEED = 0.6 #0.8
DEFAULT_PID = 0 
DEFAULT_KP = 0.01
DEFAULT_KD = 0.01


# List of Useful driving values
SPEED_STOP = 0
SPEED_GO = .6
SPEED_SLOW = .15 #.10   # 0.3 # min speed
STEER_STRAIGHT = 0
STEER_RIGHT = 25
STEER_LEFT = -18
TEST_DELAY = 2.2
FULL_TURN_DELAY = 1.8
HALF_TURN_DELAY = 1.3
INIT_DELAY = 1

class control:
    """The control class manages the connection to the jetson and driving commands/feedback
            Variables:
                serial -- serial connection to arduino
                straight -- current Straight value (1000 - 2000)
                start -- current start speed (1000 - 2000)
                init_speed -- current inital speed ( -2.0 - 2.0)
                pid_on -- pid off or on (0 or 1)
                pid_kp -- kp value
                pid_kd -- kd value

                speed -- current speed value (-2.0 - 2.0)
                steering -- current steering angle (-30 - 30)

                history -- command history file
    """

    """ Inits serial connection and pushes default calibration parameters
    """
    def __init__(self):

        self.SPEED_STOP = SPEED_STOP
        self.SPEED_GO = SPEED_GO
        self.SPEED_SLOW = SPEED_SLOW
        self.STEER_STRAIGHT = STEER_STRAIGHT
        self.STEER_RIGHT = STEER_RIGHT
        self.STEER_LEFT = STEER_LEFT
        self.TEST_DELAY = TEST_DELAY
        self.FULL_TURN_DELAY = FULL_TURN_DELAY
        self.HALF_TURN_DELAY = HALF_TURN_DELAY
        self.INIT_DELAY = INIT_DELAY

        # init everyting
        self.serial = None
        self.serial_connect()
        time.sleep(2) # This wait is needed or else the configs don't take effect
#        self.init_history()
        self.init_calibrate()
        # time.sleep(2)

        # things to be used later
        self.history = None
        self.straight = None
        self.start = None
        self.init_speed = None

        self.pid = None
        self.pid_kp = None
        self.pid_kd = None
        self.corner_turn = False
        # Make sure the car is stopped
        self.force_stop()
        time.sleep(1)

    def __del__(self):
        self.serial_close()
 #       self.history.close()

    def init_history(self):
        self.history = open(HISTORY_FILE,"w+")
        preface = '############## Driving Log Started ({})\n\n'.format(datetime.datetime.now())
        self.history.write(preface)

    ######################## CONTROL
    """Force the Car to Stop
    """
    def force_stop(self):
        self.drive(SPEED_STOP)
        self.steer(STEER_STRAIGHT)
        print('INFO: Driving Stopped')

    """Force the Car to Drive for a specific duration and steering angle
        duration -- how long to move forward (default 2)
        angle -- steering angle
        init_delay -- inital delay for pid to start moving
    """
    def force_drive(self, duration=TEST_DELAY, angle=STEER_STRAIGHT, init_delay=0):
        print('INFO: Forced Turn ({}): Starting ...'.format(angle))
        # Make sure the car is stopped
        self.force_stop()
        
        print('INFO: Delay ({})'.format(.5))
        time.sleep(.5)

        print('INFO: Turning ({})'.format(angle))
        # Start turn
        self.drive(SPEED_GO*1.2)
        self.drive(SPEED_SLOW*3) # TODO I just doubled the turn speed

        print('INFO: Delay ({})'.format(init_delay)) 
        time.sleep(init_delay)  # give it a second to pid to start
        self.steer(angle)
        print('INFO: Delay ({})'.format(duration))
        time.sleep(duration)

        # Stop
        #self.force_stop()
        #print('INFO: Force Right Turn: Finished ')
        #print('INFO: Delay ({})'.format(2))
        #time.sleep(2)

    """Force the Car to Move Forward
         duration -- how long to move forward (default 3)
    """
    def force_forward_test(self):
        self.force_drive(TEST_DELAY,STEER_STRAIGHT)

    """Force the Car to Turn Right
    """
    def force_right_turn(self):
        if self.corner_turn:
            self.force_drive(HALF_TURN_DELAY, STEER_RIGHT, INIT_DELAY*0)
        else:
            self.force_drive(HALF_TURN_DELAY, STEER_RIGHT, INIT_DELAY*1)

    """Force the Car to Turn Left
    """
    def force_left_turn(self):
        if self.corner_turn:
            self.force_drive(FULL_TURN_DELAY, STEER_LEFT, INIT_DELAY*1)
        else:
            self.force_drive(FULL_TURN_DELAY, STEER_LEFT, INIT_DELAY*.8)

    def force_road_topleft_center(self):
        self.force_drive(TEST_DELAY, STEER_STRAIGHT)
        self.force_drive(HALF_TURN_DELAY, STEER_LEFT/2)
        self.force_drive(TEST_DELAY, STEER_STRAIGHT)

    """Push Speed value
        value -- speed value (-2.0 - 2.0)
    """
    def drive(self,value):
        if not (-2.0 <= value <= 2.0):
            print('ERROR: Invalid Speed !')
            return
        command = '!speed{}\n'.format(value)
        self.push_command(command)

        # Save value

    """Push Steering value
        value -- steering angle (-30.0 - 30.0)
    """
    def steer(self, angle):
        command = '!steering{}\n'.format(angle)
        self.push_command(command)
        
        # Save value

    ######################## CALIBRATION
    """Send all Default calibration parameters
    """
    def init_calibrate(self):
        self.push_straight()
        self.push_start()
        self.push_pid()

    """Push generic command to arduino
    """
    def push_command(self, command):
        self.serial.write(command.encode())
        print('INFO: Command Sent: {}'.format(repr(command)))

        # save to command history
     #   self.history.write(command)


    ######################## STEERING
    """Send straight calibration value to arduino
        value -- steering position between 1000 and 2000
    """
    def push_straight(self, value=-1):
        # push default value if not specified
        if value == -1: value = DEFAULT_STRAIGHT
        
        # check custom value
        if not (1000 < value < 2000):  # if the value is invalid
            print('ERROR: Invalid Steering Calibration!')
            return
        
        # push custom value
        command = '!straight{}\n'.format(value)
        self.push_command(command)

        # Save parameters
        self.straight = value

    """Send starting value and speed to arduio
        value -- start speed PWM value (1000 - 2000)
        init_speed -- starting speed (-2 - 2, guess)
    """
    def push_start(self, value=-1, init_speed=-1):
        # push default values if not specified
        if value == -1:
            value = DEFUALT_START
        if init_speed == -1:
            init_speed = DEFUALT_SPEED

        # check custom inputs
        if not (1000 < value < 2000):  # if the value is invalid
            print('ERROR: Invalid Start Calibration!')
            return
        if not (-2.0 < init_speed < 2):  # if the value is invalid
            print('ERROR: Invalid Speed Calibration!')
            return
        
        # push custom values
        command = '!start{}\n!inits{}\n'.format(value,init_speed)
        self.push_command(command)

        # save parameters
        self.start = value
        self.init_speed = init_speed

    """Select and send pid parameters
        pid -- pid off or on (0 or 1)
        pid_kp -- pid kp value
        pid_kd -- pid kd value
    """
    def push_pid(self, pid=-1, kp=None, kd=None):
        # push default values if not specified
        if pid == -1:
            pid = DEFAULT_PID
        if kp is None:
            kp = DEFAULT_KP
        if kd is None:
            kd = DEFAULT_KD

        # check custom inputs
        if ((pid is not 0) and (pid is not 1)):  # if the vlaue is invalid
            print('ERROR: Invalid PID Calibration!')
            return
        
        # push custom values
        pid_str = 'pid1' if pid else 'pid0'
        command = '!kp{}\n!kd{}\n!{}\n'.format(kp,kd,pid_str)
        self.push_command(command)

        # save parameters
        self.pid = pid
        self.pid_kp = kp
        self.pid_kd = kd

    """Request encoder value and read result
    """
    def get_encoder(self):
        self.push_command('!getEncoder\n')
        time.sleep(.001)
        value = self.serial.readline()
        #print(value)
        return round(float(value.decode()))

    def get_speed(self):
        self.push_command('!getSpeed\n')
        time.sleep(.001)
        value = self.serial.readline()
        return round(float(value.decode()),2)

    def get_distance(self):
        self.push_command('!distance\n')
        time.sleep(.001)
        value = self.serial.readline()
        return round(float(value.decode()),2)
        

    ######################## SERIAL CONNECTION

    """Sets up Serial Connection to Arduio 
    """
    def serial_connect(self):
        self.serial = serial.Serial(ARDUINO_SERIAL, 115200, timeout=.5)     # Assumed Baud rate
        #self.serial.flushInput()    # This might make the wheels turn full left every so often
        #self.serial.flushOutput()
        print('INFO: Arduino Connection Established')

    """Read a number from Serial
    """
    def serial_read_num(self):
        try:
            while True:
                line = self.serial.readline()
                print(line) 
                if line:
                    num = float(line)
                    return num
        except Exception as e:
            raise e
        return -1

    """Closes Serial Connection to Arduio 
        TODO :  figure out what needs to be done to prevent wheel from turning full left
    """
    def serial_close(self):
        #self.serial.flushInput()    # This might prevent the wheels from turning
        #self.serial.flushOutput()
        self.serial.close()
        print('INFO: Arduino Connection Closed')
