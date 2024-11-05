#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
#from .arrg_utils.sysinfo import SysInfo
from sys_info.sysinfo_lib import SysInfo
from std_msgs.msg import String


class SysInfoNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.__sys_info = SysInfo()
        self.__sys_info_pub = self.create_publisher(String, '/sys_info', 10)
        self.__sys_info_timer = self.create_timer(2.0, self.__on_sys_info_clbk)

    def __on_sys_info_clbk(self):
        sys_info_msg = String()
        retVal = self.__sys_info.get_system_snapshot()
        sys_info_msg.data = str(retVal) #f"host ip: {retVal['ip']}" 
        # self.get_logger().info(self.__sys_info.get_system_snapshot()) 
        self.__sys_info_pub.publish(sys_info_msg)

def main(args=None):
    rclpy.init(args=args)
    sys_info_node = SysInfoNode('sys_info_node')
    rclpy.spin(sys_info_node)
    rclpy.shutdown()


if __name__=="__main__":
    main