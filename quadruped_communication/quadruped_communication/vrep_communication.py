#!/usr/bin/env python3

from time import sleep
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
import cv2
import numpy as np

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
                
                # Convertir los datos binarios a array de bytes
                if isinstance(image, bytes):
                    img_bytes = np.frombuffer(image, dtype=np.uint8)
                else:
                    img_bytes = np.array(image, dtype=np.uint8)

                # Reshape considerando que los datos están en formato RGB
                try:
                    img_array = img_bytes.reshape((resolution[1], resolution[0], 3))
                except ValueError as e:
                    self.get_logger().error(f"Error al reshape de la imagen: {e}")
                    self.get_logger().info(f"Tamaño del buffer: {len(img_bytes)}, Resolución esperada: {resolution}")
                    return
                # Redimensionar la imagen
                desired_width = 600
                desired_height = 450
                img_resized = cv2.resize(img_array, (desired_width, desired_height), 
                                    interpolation=cv2.INTER_LINEAR)

                # Crear mensaje ROS
                msg = Image()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_link'
                msg.height = desired_height
                msg.width = desired_width
                msg.encoding = 'rgb8'
                msg.is_bigendian = False
                msg.step = 3 * desired_width
                msg.data = img_resized.tobytes()  # Usar tobytes() en lugar de list()

                self.publisher_vrep_cam.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Error al procesar la imagen: {e}")

        self.create_timer(0.1, publish_image)


def main(args=None):
    rclpy.init(args=args)
    node = Vrep_Communication("vrep_communication")
    rclpy.spin(node)
    node.sim.stopSimulation()
    node.destroy_node()

    rclpy.shutdown()