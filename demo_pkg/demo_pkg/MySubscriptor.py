#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MySecNodo(Node):
    def __init__(self, node_name):
        super().__init__(node_name=node_name)
        self._topic_sub = self.create_subscription(String, '/mensaje', self._subs_callback, 10)

    def _subs_callback(self, msg):
        self.get_logger().info(f"Recibi ({msg.data})")

def main(args=None):
    rclpy.init(args=args)
    sub_nodo = MySecNodo('MySegNodo')
    rclpy.spin(sub_nodo)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
