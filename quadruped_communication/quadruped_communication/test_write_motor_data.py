#!/usr/bin/env python3

import os
import numpy as np
from rclpy.node import Node
import rclpy
from quadruped_interfaces.msg import MotorData
from sensor_msgs.msg import JointState

# Globales para manejo de archivos
ruta_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'motor_data')
new_file = True
number = 0


class MotorDataWriter(Node):
    """
    Nodo para escribir datos del mensaje `MotorData` en archivos.
    """

    def __init__(self):
        super().__init__("write_motor_data_node")

        self.joint_subscription   = self.create_subscription(JointState, '/joint_goals', self.callback, 10)
        self.publisher_motor_data = self.create_publisher   (MotorData,  '/motor_data' , 10)

        self.get_logger().warn("The test_write_motor_data_node has been started")

        self.create_timer(0.1, self.update_data)
        self.motor_position_converted = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self.present_pos = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self.motors = MotorData()

    def update_data(self):
        for i in range(len(self.present_pos)):
            self.present_pos[i]+=0.1

        self.motors.goal_position = self.present_pos
        self.motors.pres_position = [self.motor_position_converted[0], self.motor_position_converted[1], self.motor_position_converted[2], self.motor_position_converted[3], self.motor_position_converted[4], self.motor_position_converted[5], self.motor_position_converted[6], self.motor_position_converted[7], self.motor_position_converted[8]]
        self.publisher_motor_data.publish(self.motors)


    def callback(self, data: JointState):
        
        self.motor_position_converted[0] = data.position[0]*180/3.1416 + 180
        self.motor_position_converted[1] = data.position[1]*180/3.1416
        self.motor_position_converted[2] = data.position[2]*180/3.1416
        self.motor_position_converted[3] = data.position[3]*180/3.1416
        self.motor_position_converted[4] = data.position[4]*180/3.1416
        self.motor_position_converted[5] = data.position[5]*180/3.1416
        self.motor_position_converted[6] = data.position[6]*180/3.1416
        self.motor_position_converted[7] = data.position[7]*180/3.1416
        self.motor_position_converted[8] = data.position[8]*180/3.1416


def main(args=None):
    """
    Función principal para inicializar el nodo y mantenerlo en ejecución.
    """
    rclpy.init(args=args)
    node = MotorDataWriter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down write_motor_data_node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
