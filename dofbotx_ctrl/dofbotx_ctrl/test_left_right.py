#!/usr/bin/env python3
#coding=utf-8
import time
from Arm_Lib import Arm_Device

Arm = Arm_Device()
time.sleep(.1)

# Cycle control the mechanical arm to swing up, down, left and right
def main():
    # Reset and center the steering gear
    Arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, 500)
    time.sleep(1)


    while True:
        # Control the up and down operation of No. 3 and No. 4 steering gear
        Arm.Arm_serial_servo_write(3, 0, 1000)
        time.sleep(.001)
        Arm.Arm_serial_servo_write(4, 180, 1000)
        time.sleep(1)
        
        # Control the left and right movement of No. 1 steering gear
        Arm.Arm_serial_servo_write(1, 180, 500)
        time.sleep(.5)
        Arm.Arm_serial_servo_write(1, 0, 1000)
        time.sleep(1)
        
        # Control the steering gear to return to the initial position
        Arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, 1000)
        time.sleep(1.5)


try :
    main()
except KeyboardInterrupt:
    print(" Program closed! ")
    pass

