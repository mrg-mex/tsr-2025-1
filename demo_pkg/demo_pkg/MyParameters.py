#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.parameter import Parameter

class ParamSrvr(Node):
    def __init__(self, node_name):
        super().__init__(node_name=node_name)
        # declarar parametros utilizando el metodo 'declare_parameter'
        # declaramos un solo parametro llamado 'timer_period'

        self.declare_parameter(
            name='timer_period', 
            value=0.0,
            descriptor=ParameterDescriptor(description = 'Configuracion del timer de muestreo')
        )
        self.declare_parameters(
            namespace='',
            parameters=[
                ('lin_vel', 0.1),
                ('ang_vel', 0.04),
                ('label_str', rclpy.Parameter.Type.STRING),
                ('param_int', rclpy.Parameter.Type.INTEGER),
                ('controller', [0.5, 0.7, 1000.0])
            ]
        )

        self._timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        self.add_on_set_parameters_callback(self._on_parameter_change)

        self.get_logger().info("ParamSrvr inicializdo...")

    def _on_parameter_change(self, params: list[Parameter]):
        success = True
        for param in params:
            if param.name == 'timer_period':
                if param.value < 0:
                    success = False
                    self.get_logger().warn(f"El parametro '{param.name} no puede ser negativo'")
            else:
                self.get_logger().info(f"El parametro '{param.name} no es monitoreado'")

        result_msg = SetParametersResult()
        result_msg.successful = success

        return result_msg

def main(args=None):
    rclpy.init(args=args)
    nodo = ParamSrvr('myparams')
    rclpy.spin(nodo)

    rclpy.shutdown()

if __name__ == '__main__':
    main()    
