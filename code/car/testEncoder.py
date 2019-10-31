

import sys
import os
import serial
import time

sys.path.append(os.path.abspath("../car/"))                                                                                                                              
from driving import control


ctl = control()


ctl.drive(ctl.SPEED_GO)
ctl.drive(ctl.SPEED_SLOW)

while True:
    #ctl.push_command("!distance\n")
    #time.sleep(.01)
    #ctl.serial.flushInput()
    #num = ctl.serial.readline()
    #ctl.push_command('!distance')
    #num = ctl.serial.readline()
    time.sleep(1)
    num = ctl.get_speed()
    #ctl.push_command('!getEncoder')
    print(num)

