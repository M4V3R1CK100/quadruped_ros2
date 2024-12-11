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
        super().__init__("gazebo_communication")
        self.get_logger().info("Nodo creado")
        self.joint_states_pub = self.create_subscription(JointState, 'joint_states', 10)
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.joint_msg = JointTrajectory()
        self.joint_msg.joint_names = [
            'camera_joint',
            'front_right_joint1',
            'front_right_joint2',
            'front_left_joint1',
            'front_left_joint2',
            'back_left_joint1',
            'back_left_joint2',
            'back_right_joint1',
            'back_right_joint2'
        ]
        self.point = JointTrajectoryPoint()    
        self.point.time_from_start.sec = 2 # Movimiento en 2 segundos
        
    def send_joint_trajectory(self,msg):
        # Configurar las posiciones deseadas y el tiempo de movimiento
        self.get_logger().warn(msg)
        
        # self.joint_msg.points.append(msg.position)
        
        # # Publicar el mensaje
        # self.publisher.publish(self.joint_msg)

    
def main(args=None):
    rclpy.init(args=args)
    movement_node = MyNode()

    #Home(movement_node)
    
    # Crear un hilo para user_input_loop
    # user_input_loop(movement_node)

    # Ejecutar el nodo ROS
    rclpy.spin(movement_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()