#!/usr/bin/env python3
#coding=utf-8

import time
from Arm_Lib import Arm_Device

# Gets the object of the manipulator
Arm = Arm_Device()
time.sleep(.1)

# The buzzer will automatically sound and turn off after 100 milliseconds
b_time = 1
Arm.Arm_Buzzer_On(b_time)
time.sleep(1)

# The buzzer will automatically sound and turn off after 300 milliseconds
b_time = 3
Arm.Arm_Buzzer_On(b_time)
time.sleep(1)

# The buzzer keeps ringing
Arm.Arm_Buzzer_On()
time.sleep(1)

# Turn off the buzzer
Arm.Arm_Buzzer_Off()
time.sleep(1)

# Release arm object
print(" Program closed! ")
del Arm


