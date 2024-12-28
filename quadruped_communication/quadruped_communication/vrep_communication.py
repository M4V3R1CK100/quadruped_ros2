#!/usr/bin/env python3

from time import sleep
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node


class Vrep_Communication(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"{name} creado")
        self.joint_subscription = self.create_subscription(JointState, '/joint_goals', self.listener_callback, 10)
        self.publisher_vrep_cam = self.create_publisher(Image, '/camera_link/image_raw', 10)

        # Handles
        self.joint_handles = []

        # Crear cliente y conectarse al servidor
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.scriptHandle = self.sim.getObject('/moveJoints')

        try:
            # Comprobar conexión
            self.sim.startSimulation()
            self.get_logger().warn("Conexión establecida con el servidor.")

        except Exception as e:
            self.get_logger().error(f"Error durante la ejecución: {e}")

        self.camera_vision()
        

    def listener_callback(self, msg: JointState):

        # Llamar a la función del script en CoppeliaSim
        success = self.sim.callScriptFunction('sysCall_joint', self.scriptHandle, list(msg._position))

    def camera_vision(self):
        sensor1Handle = self.sim.getObject('/camera')
        
        if sensor1Handle == -1:
            self.get_logger().error("No se encontró el sensor de visión '/camera'")
            return

        def publish_image():
            try:
                image, resolution = self.sim.getVisionSensorImg(sensor1Handle)

                if image is None or resolution is None:
                    self.get_logger().warn("No se pudo obtener imagen del sensor.")
                    return

                msg = Image()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_link'
                msg.height = resolution[1]
                msg.width = resolution[0]
                msg.encoding = 'rgb8'
                msg.is_bigendian = False
                msg.step = 3 * resolution[0]
                msg.data = list(image)

                self.publisher_vrep_cam.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Error al procesar la imagen: {e}")

        self.create_timer(0.05, publish_image)


def main(args=None):
    rclpy.init(args=args)
    node = Vrep_Communication("vrep_communication")
    rclpy.spin(node)
    node.sim.stopSimulation()
    node.destroy_node()

    rclpy.shutdown()