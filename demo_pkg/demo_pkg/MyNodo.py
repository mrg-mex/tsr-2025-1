#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from my_interface.msg import MyPosition, TargetPosition


class MyNodo(Node):
    def __init__(self):
        super().__init__('my_node')
        self._topic_pub = self.create_publisher(MyPosition, '/mensaje', 10)
        self.target_pos_pub = self.create_publisher(TargetPosition, '/target_pos', 10)
        self._timer = self.create_timer(3.0, self._timer_callback)
        self._target_timer = self.create_timer(1.0, self._pub_trg_pos_clbk)
        self._count = 0
        self.get_logger().info("Nodo creado. ")
        
    def _pub_trg_pos_clbk(self):
        target_msg = TargetPosition()
        # Header del mensaje
        target_msg.header.frame_id = ''
        target_msg.header.stamp = self.get_clock().now().to_msg()
        # Etiqueta del punto destino
        target_msg.destino = 'Punto destino'
        # Variables de la posicion destino
        target_msg.target_point.x = self._count / 5
        target_msg.target_point.y = self._count / 2
        target_msg.target_point.z = self._count / 3
        self.target_pos_pub.publish(target_msg)

    def _timer_callback(self):
        self._count += 1.0
        mensaje = MyPosition()
        mensaje.etiqueta = f"({self._count}) Repeticiones cada 3seg."
        mensaje.pos_x = self._count
        mensaje.pos_y = self._count / 3
        mensaje.pos_z = self._count / 4
        self._topic_pub.publish(mensaje)

def main(args=None):
    rclpy.init(args=args)
    mynodo = MyNodo()
    rclpy.spin(mynodo)
    rclpy.shutdown()

if __name__ == '__main__':
    main()