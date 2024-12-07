#!/usr/bin/env python3
#coding=utf-8
import time
from Arm_Lib import Arm_Device

Arm = Arm_Device()
time.sleep(.1)

# Control the motion of six steering gears at the same time and gradually change the angle
def ctrl_all_servo(angle, s_time = 500):
    Arm.Arm_serial_servo_write6(angle, 180-angle, angle, angle, angle, angle, s_time)
    time.sleep(s_time/1000)


def main():
    dir_state = 1
    angle = 90
    
    # Reset and center the steering gear
    Arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, 500)
    time.sleep(1)

    
    while True:
        if dir_state == 1:
            angle += 1
            if angle >= 180:
                dir_state = 0
        else:
            angle -= 1
            if angle <= 0:
                dir_state = 1
        
        ctrl_all_servo(angle, 10)
        time.sleep(10/1000)
        # print(angle)

    
try :
    main()
except KeyboardInterrupt:
    print(" Program closed! ")
    pass

