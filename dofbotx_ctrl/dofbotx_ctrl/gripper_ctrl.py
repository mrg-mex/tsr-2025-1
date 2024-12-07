#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from Arm_Lib import Arm_Device
import time

class GripperCtrl(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self__gripper_cmd_sub = self.create_subscription(String, '/gripper_cmd', self._on_gripper_cmd, self.qos_profile)
        self.__gripper_states = ['open', 'close']
        self.__gripper_pos = [50, 180]
        self._gripper_state = ''
        self._is_busy = False
        self._default_delay = 500       # en millis
        self.__dofbot_arm = Arm_Device()

    def _move_gripper(self, cmd, delay):
        self._is_busy = True
        indx = self.__gripper_states.index(cmd)
        self.__dofbot_arm.Arm_serial_servo_write(6, self.__gripper_pos[indx], self._default_delay)
        time.sleep(1.0)
        self._gripper_state = cmd
        self._is_busy = False


    def _on_gripper_cmd(self, msg:String):
        istate = msg.data.lower()
        if istate in self.__gripper_states:
            if (not istate != self._gripper_state) and (not self._is_busy):
                self._move_gripper(istate, self._default_delay)



def main(args=None):
    pass

if __name__ == "__main__":
    main()