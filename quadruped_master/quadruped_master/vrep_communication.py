#!/usr/bin/env python3

from time import sleep
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node


class Vrep_Communication(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"{name} creado")
        self.joint_subscription = self.create_subscription(JointState, '/joint_goals', self.listener_callback, 10)

        # Handles
        self.joint_handles = []

        # Crear cliente y conectarse al servidor
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')

        try:
            # Comprobar conexión
            self.sim.startSimulation()
            self.get_logger().warn("Conexión establecida con el servidor.")

        except Exception as e:
            self.get_logger().error(f"Error durante la ejecución: {e}")
        

    def listener_callback(self, msg: JointState):
        print("joa")
        joint_goals = msg
        self.get_logger().info(f"{joint_goals._name}")

        if len(self.joint_handles) ==  0:
            for joint in joint_goals._name:
                handle = self.sim.getObject(f"/{joint}")
                if handle != -1:
                    self.joint_handles.append(handle) #se busca el handle del objeto a partir de su nombre, el handle es un número
                    self.get_logger().info(f"Handle de {joint}: {handle}")
                else:
                    self.get_logger().error(f"No se encontró handle para {joint}")

        for i in range(len(self.joint_handles)):
            self.sim.setJointTargetPosition(self.joint_handles[i], joint_goals._position[i])

def main(args=None):
    rclpy.init(args=args)
    node = Vrep_Communication("vrep_communication")
    rclpy.spin(node)
    node.destroy_node()
    node.sim.stopSimulation()

    rclpy.shutdown()