#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from quadruped_interfaces.msg import MotionParams



class Joy_Control(Node):
    def __init__(self, name):
        super().__init__(name)
        self.motion_publisher_   = self.create_publisher(MotionParams, 'motion_params', 10)
        self.joy_subscription = self.create_subscription(Joy, '/joy', self.listener_callback, 10)


        self.motion_quadruped = MotionParams()

        self.motion_quadruped.speed        = 0.0  # Por ejemplo, velocidad de 1.0 m/s
        self.motion_quadruped.rotation     = 0.0  # Por ejemplo, rotación de 0.5 radianes
        self.motion_quadruped.traslation_x = 0.0  # Por ejemplo, traslación de 2.0 metros
        self.motion_quadruped.traslation_z = 0.0  # Por ejemplo, traslación de 2.0 metros
        self.motion_quadruped.motion       = 0.0  # Por ejemplo, traslación de 2.0 metros
        
        
    def listener_callback(self, msg):

        
        
        self.motion_quadruped.speed        = self.motion_quadruped.speed + 0.1 if (msg.buttons[1]) else self.motion_quadruped.speed
        self.motion_quadruped.rotation     = self.motion_quadruped.rotation
        self.motion_quadruped.traslation_x = self.motion_quadruped.traslation_x
        self.motion_quadruped.traslation_z = self.motion_quadruped.traslation_z
        self.motion_quadruped.motion       = self.motion_quadruped.motion


        self.motion_publisher_.publish(self.motion_quadruped)
        self.get_logger().info("Sending msg")

            
def main(args=None):
    rclpy.init(args=args)
    node = Joy_Control("joy_to_motion_node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()