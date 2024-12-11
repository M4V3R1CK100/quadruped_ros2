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
        super().__init__("gazebo_communication_node")
        self.get_logger().info("Nodo creado")
        self.joint_states_pub = self.create_subscription(JointState, 'joint_goals',self.send_joint_trajectory, 10)
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.joint_names = [
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
        
    def send_joint_trajectory(self,msg):
        # Crear un nuevo mensaje de trayectoria
        joint_msg = JointTrajectory()
        joint_msg.joint_names = self.joint_names

        # Crear un nuevo punto de trayectoria
        point = JointTrajectoryPoint()
        point.positions = msg.position  # Posiciones deseadas
        point.time_from_start.sec = 2   # Movimiento en 2 segundos

        # Añadir el punto al mensaje de trayectoria
        joint_msg.points.append(point)

        # Publicar el mensaje
        self.publisher.publish(joint_msg)
        self.get_logger().info("Trayectoria publicada: %s" % str(msg.position))

    

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