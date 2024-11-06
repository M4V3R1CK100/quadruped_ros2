#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from quadruped_interfaces.msg import MotionParams

class my_node(Node):
    def __init__(self):
        super().__init__('interface_node')
        
        self.pub = self.create_publisher(MotionParams, 'motion_params', 10)

        self.motion_params = MotionParams()

        self.timer = self.create_timer(10, self.timer_callback)

        self.get_logger().info("Interface Node initialized")

        self.motion_params.speed = 1.0  # Por ejemplo, velocidad de 1.0 m/s
        self.motion_params.rotation = 0.0  # Por ejemplo, rotación de 0.5 radianes
        self.motion_params.traslation_x = 0.0  # Por ejemplo, traslación de 2.0 metros
        self.motion_params.traslation_z = 0.0  # Por ejemplo, traslación de 2.0 metros
        self.motion_params.motion = 0.0  # Por ejemplo, traslación de 2.0 metros
    
    def timer_callback(self):

        # Asignar valores a los campos de MotionParams (ajusta según la estructura del mensaje)
        self.motion_params.speed = 1.0  # Por ejemplo, velocidad de 1.0 m/s
        self.motion_params.rotation = 0.0  # Por ejemplo, rotación de 0.5 radianes
        self.motion_params.traslation_x = 0.0  # Por ejemplo, traslación de 2.0 metros
        self.motion_params.traslation_z = 0.0  # Por ejemplo, traslación de 2.0 metros
        self.motion_params.motion = 0.0  # Por ejemplo, traslación de 2.0 metros

        # Publicar el mensaje en el tema 'motion_params'
        self.pub.publish(self.motion_params)



def main(args=None):
    rclpy.init(args=args)
    node = my_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()