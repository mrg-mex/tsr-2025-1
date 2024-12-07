#!/usr/bin/env python3
#coding=utf-8
import time
from Arm_Lib import Arm_Device

Arm = Arm_Device()
time.sleep(.1)
# Read the angles of all steering gears and print them out circularly
def main():

    while True:
        for i in range(6):
            aa = Arm.Arm_serial_servo_read(i+1)
            print(aa)
            time.sleep(.01)
        time.sleep(.5)
        print(" END OF LINE! ")

    
try :
    main()
except KeyboardInterrupt:
    print(" Program closed! ")
    pass
