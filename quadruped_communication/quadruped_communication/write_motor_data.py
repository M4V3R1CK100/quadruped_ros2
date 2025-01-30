#!/usr/bin/env python3

import os
import numpy as np
from rclpy.node import Node
import rclpy
from quadruped_interfaces.msg import MotorData


class MotorDataWriter(Node):
    """
    Nodo para escribir datos del mensaje `MotorData` en archivos.
    """

    def __init__(self):
        super().__init__("write_motor_data_node")

        # Suscriptor para el tópico /motor_data
        self.create_subscription(MotorData, "/motor_data", self.save_data, 10)

        self.get_logger().warn("The write_motor_data_node has been started")

        self.ruta_file = os.path.join(os.getcwd(), 'src', 'quadruped_ros2' , 'motor_data')
        print(self.ruta_file)
        self.new_file = True
        self.number = 0


    def save_data(self, motor: MotorData):
        """
        Callback para guardar los datos recibidos en los archivos correspondientes.
        """

        self.write_data("goal_position", ",".join(map(str, motor.goal_position)) + "\n")
        self.write_data("pres_position", ",".join(map(str, motor.pres_position)) + "\n")

    def write_data(self, name: str, data: str):
        """
        Escribe datos en un archivo con nombre único, basado en el número de archivo.
        """
        while True:
            file_path = os.path.join(self.ruta_file, f"{self.number}_{name}.txt")
            if os.path.exists(file_path):
                if self.new_file:
                    self.number += 1
                else:
                    with open(file_path, "a") as archivo:
                        archivo.write(data)
                    break
            else:
                os.makedirs(self.ruta_file, exist_ok=True)
                with open(file_path, "w") as archivo:
                    archivo.write(data)
                self.new_file = False
                break



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
        # rclpy.shutdown()


if __name__ == "__main__":
    main()
