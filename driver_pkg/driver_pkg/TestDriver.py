#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# from Rosboard_Lib import Rosboard
from driver_interface.msg import BuzzerCtrl

class BuzzerNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        # self.__rosboar_drv = Rosboard()
        self.__buzzer_sub = self.create_subscription(
            BuzzerCtrl, 
            '/buzzer_ctrl', 
            self.__on_buzzer_clbk, 
            10
            )

    def __on_buzzer_clbk(self, msg: BuzzerCtrl):
        buzzerCmd = msg
        self.play_buzzer(buzzerCmd.on_time)

    def play_buzzer(self, on_time: int):
        # self.__rosboar_drv.set_beep(on_time)
        self.get_logger().info(f"Current Buzzer 'on_time' = {on_time}")

def main(args=None):
    rclpy.init(args=args)
    buzzer_node = BuzzerNode('buzzer_node')
    rclpy.spin(buzzer_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
