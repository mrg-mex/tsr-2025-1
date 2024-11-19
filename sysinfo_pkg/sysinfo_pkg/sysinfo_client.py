#!/usr/bin/env python3

import rclpy
import sys

from rclpy.node import Node
from my_interface.srv import SysInfo


class SysInfoClient(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self._sysinfo_client = self.create_client(SysInfo, '/sysinfo_svc',)
        while not self._sysinfo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("servicio no disponible, esperando por el servicio...")
        self.get_logger().info(f"{node_name} service client inicializado.")

    def call_server(self, cmd:str):
        peticion = SysInfo.Request()
        peticion.modo = cmd
        self.future = self._sysinfo_client.call_async(peticion)
        rclpy.spin_until_future_complete(self, future=self.future)

        return self.future.result()

def eval_response(success, msg, status_msg):
    if success:
        print(f"El proceso fue exitoso {status_msg}")
        print(msg)
    else:
        print(f"El proceso no fue exitoso {msg}")


def main(args=None):
    rclpy.init()
    svc_client_node = SysInfoClient('cliente_servicio')
    cmd = sys.argv[1].lower()
    resultado = svc_client_node.call_server(cmd=cmd)
    print(resultado.success) 
    print(resultado.sysinfo_msg)
    print(resultado.string_status_message)
    #svc_client_node.call_server('snapshot')
    #svc_client_node.call_server('partial')
    rclpy.shutdown()

if __name__ == '__main__':
    main()