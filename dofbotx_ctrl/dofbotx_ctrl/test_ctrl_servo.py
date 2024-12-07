#!/usr/bin/env python3
#coding=utf-8

import time
from Arm_Lib import Arm_Device

Arm = Arm_Device()
time.sleep(.1)

# Individually control a steering gear to move to a certain angle
id = 6

Arm.Arm_serial_servo_write(id, 90, 500)
time.sleep(1)

# Control a steering gear cycle switching angle
id = 6

def main():
    while True:
        Arm.Arm_serial_servo_write(id, 120, 500)
        time.sleep(1)
        Arm.Arm_serial_servo_write(id, 50, 500)
        time.sleep(1)
        Arm.Arm_serial_servo_write(id, 120, 500)
        time.sleep(1)
        Arm.Arm_serial_servo_write(id, 180, 500)
        time.sleep(1)


try :
    main()
except KeyboardInterrupt:
    print(" Program closed! ")
    pass
