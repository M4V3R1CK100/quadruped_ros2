#!/usr/bin/env python3
import rclpy
import sys
import threading
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
from math import pi, cos, sin, acos, atan2
from rclpy.node import Node
import time
from quadruped_master.gait_functions import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MyNode(Node):
    def __init__(self):
        super().__init__("copplelia_communication_node")
        self.get_logger().info("Nodo creado")
        self.joint_goals_sub = self.create_subscription(JointState, 'joint_goals',self.send_joint_trajectory, 10)
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        self.joint_states = JointState()
        self.joint_states.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
        self.joint_states.header = Header()
        self.joint_states.name = ['front_right_joint1', 'front_right_joint2', 'front_left_joint1', 'front_left_joint2', 
                                  'back_left_joint1', 'back_left_joint2','back_right_joint1', 'back_right_joint2']
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.joint_states)

    def send_joint_trajectory(self,msg):
        # Publicar el mensaje
        self.joint_states.header.stamp = self.get_clock().now().to_msg()  # Actualiza el timestamp antes de publicar
        self.joint_states.position =  msg.position[1:]  # Posiciones deseadas
        self.publisher.publish(self.joint_states)
    

def main(args=None):
    rclpy.init(args=args)
    movement_node = MyNode()

    try:
        rclpy.spin(movement_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()