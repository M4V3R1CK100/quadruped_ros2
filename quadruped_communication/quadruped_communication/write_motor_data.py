#!/usr/bin/env python3

import os
import numpy as np
from rclpy.node import Node
from rclpy import init, spin
from archie_master.msg import MotorData

# Globales para manejo de archivos
ruta_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'motor_data')
new_file = True
number = 0


def write_data(name: str, data: str):
    """
    Escribe datos en un archivo con nombre único, basado en el número de archivo.
    """
    global new_file, number
    while True:
        file_path = os.path.join(ruta_file, f"{number}_motor_{name}.txt")
        if os.path.exists(file_path):
            if new_file:
                number += 1
            else:
                with open(file_path, "a") as archivo:
                    archivo.write(data)
                break
        else:
            with open(file_path, "w") as archivo:
                archivo.write(data)
            new_file = False
            break


class MotorDataWriter(Node):
    """
    Nodo para escribir datos del mensaje `MotorData` en archivos.
    """

    def __init__(self):
        super().__init__("write_motor_data_node")

        # Suscriptor para el tópico /motor_data
        self.create_subscription(MotorData, "/motor_data", self.save_data, 10)

        self.get_logger().warn("The write_motor_data_node has been started")

    def save_data(self, motor: MotorData):
        """
        Callback para guardar los datos recibidos en los archivos correspondientes.
        """
        write_data("goal_position", ",".join(map(str, motor.goal_position)) + "\n")
        write_data("pres_position", ",".join(map(str, motor.pres_position)) + "\n")


def main(args=None):
    """
    Función principal para inicializar el nodo y mantenerlo en ejecución.
    """
    init(args=args)
    node = MotorDataWriter()
    try:
        spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down write_motor_data_node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
