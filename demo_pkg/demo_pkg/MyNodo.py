#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNodo(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info("Nodo creado. ")
        

def main():
    pass

if __name__ == '__main__':
    main()