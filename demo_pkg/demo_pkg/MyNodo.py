#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyNodo(Node):
    def __init__(self):
        super().__init__('my_node')
        self._topic_pub = self.create_publisher(String, '/mensaje', 10)
        self._timer = self.create_timer(3.0, self._timer_callback)
        self._count = 0
        self.get_logger().info("Nodo creado. ")
        
    def _timer_callback(self):
        self._count += 1
        mensaje = String()
        mensaje.data = f"({self._count}) Repeticiones cada 3seg."
        self._topic_pub.publish(mensaje)

def main(args=None):
    rclpy.init(args=args)
    mynodo = MyNodo()
    rclpy.spin(mynodo)
    rclpy.shutdown()

if __name__ == '__main__':
    main()