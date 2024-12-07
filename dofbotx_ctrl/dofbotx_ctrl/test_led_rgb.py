import time
from Arm_Lib import Arm_Device

# Gets the object of the manipulator
Arm = Arm_Device()
time.sleep(.1)


def main():
    
    while True:
        Arm.Arm_RGB_set(50, 0, 0) # RGB red
        time.sleep(.5)
        Arm.Arm_RGB_set(0, 50, 0) # RGB green
        time.sleep(.5)
        Arm.Arm_RGB_set(0, 0, 50) # RGB blue
        time.sleep(.5)

        print(" END OF LINE! ")

try :
    main()
except KeyboardInterrupt:
    # Release arm object 
    del Arm
    print(" Program closed! ")
    pass

