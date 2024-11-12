#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_interface.srv import SysInfo

class SysInfoSrv(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.__server = self.create_service(SysInfo, 
            '/sysinfo_svc', self.__on_service_call)

    def __on_service_call(self, request:SysInfo):
        modo = str(request.Request.modo)
        respuesta = SysInfo.Response()
        respuesta.success = True
        respuesta.string_status_message = ""

        if modo.lower() == "full":
            respuesta.string_status_message = f"El modo seleccionado es correcto: {modo}."
        elif modo.lower() == "snapshot":
            respuesta.string_status_message = f"El modo seleccionado es correcto: {modo}."
        else:
            respuesta.success = False
            respuesta.string_status_message = f"No recozco el modo {modo}."

        return respuesta

def main(args=None):
    pass

if __name__ == '__main__':
    main()