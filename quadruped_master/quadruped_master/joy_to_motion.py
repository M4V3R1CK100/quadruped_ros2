#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from quadruped_interfaces.msg import MotionParams
import time



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
        
        
    def listener_callback(self, msg: Joy):
        
        self.motion_quadruped.speed        = (round(self.motion_quadruped.speed, 1) + 0.1 if (msg.buttons[1] and (self.motion_quadruped.speed <  1.0)) else
                                             (round(self.motion_quadruped.speed, 1) - 0.1 if (msg.buttons[2] and (self.motion_quadruped.speed > -1.0)) else 
                                              round(self.motion_quadruped.speed, 1)))
        self.motion_quadruped.rotation     = (round(self.motion_quadruped.rotation, 1) + 5.0 if (msg.buttons[3] and (self.motion_quadruped.rotation <  30.0)) else
                                             (round(self.motion_quadruped.rotation, 1) - 5.0 if (msg.buttons[0] and (self.motion_quadruped.rotation > -30.0)) else 
                                              round(self.motion_quadruped.rotation, 1)))

        self.motion_quadruped.traslation_x = (round(self.motion_quadruped.traslation_x - 0.05*msg.axes[6], 2)) if ((self.motion_quadruped.traslation_x - 0.05*msg.axes[6]<0.245) and (self.motion_quadruped.traslation_x - 0.05*msg.axes[6]>-0.245)) else self.motion_quadruped.traslation_x
                                             
        self.motion_quadruped.traslation_z = (round(self.motion_quadruped.traslation_z + 0.05*msg.axes[7], 2)) if ((self.motion_quadruped.traslation_z + 0.05*msg.axes[7]<0.245) and (self.motion_quadruped.traslation_z + 0.05*msg.axes[7]>-0.245)) else self.motion_quadruped.traslation_z

        self.motion_quadruped.motion       = self.motion_quadruped.motion


        # self.motion_quadruped.speed        = 0.1 if (msg.buttons[1]) else (-0.1 if (msg.buttons[3]) else 0.0)
        # self.motion_quadruped.rotation     = 0.1 if (msg.buttons[2]) else (-0.1 if (msg.buttons[0]) else 0.0)
        # self.motion_quadruped.traslation_x = 0.1*msg.axes[6]
        # self.motion_quadruped.traslation_z = 0.1*msg.axes[7]
        # self.motion_quadruped.motion       = self.motion_quadruped.motion


        self.get_logger().info(str(self.motion_quadruped))        

        if (msg.buttons[10]):
            self.motion_publisher_.publish(self.motion_quadruped)
            time.sleep(1.5)

        time.sleep(0.2)

            
def main(args=None):
    rclpy.init(args=args)
    node = Joy_Control("joy_to_motion_node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()