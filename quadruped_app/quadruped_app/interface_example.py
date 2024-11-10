#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from quadruped_interfaces.msg import MotionParams
import flet as ft
import threading

class MyNode(Node):
    def __init__(self):
        super().__init__('interface_node')
        
        self.pub = self.create_publisher(MotionParams, 'motion_params', 10)
        self.motion_params = MotionParams()

        # Configurar un temporizador con un intervalo de 1 segundo
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Interface Node initialized")

    def timer_callback(self):
        # Asignar valores a los campos de MotionParams
        self.motion_params.speed = 1.0
        self.motion_params.rotation = 0.0
        self.motion_params.traslation_x = 0.0
        self.motion_params.traslation_z = 0.0
        self.motion_params.motion = 0.0

        # Publicar el mensaje en el tema 'motion_params'
        self.pub.publish(self.motion_params)
        self.get_logger().info("Published motion parameters")

def ros_spin(node):
    # Este método se ejecutará en un hilo separado
    rclpy.spin(node)

def interface(page: ft.Page):
    page.title = "Quadruped Controller"
    page.vertical_alignment = ft.MainAxisAlignment.CENTER
    page.window.height = 750
    page.window.width = 900
    page.auto_scroll = True
    page.scroll = ft.ScrollMode.HIDDEN
    page.theme_mode = "dark"
    page.window.min_width = 700
    page.window.min_height = 700

    page.theme = ft.Theme(
        color_scheme_seed=ft.colors.BLUE,
        visual_density=ft.VisualDensity.COMFORTABLE,
        color_scheme=ft.ColorScheme(
            primary=ft.colors.BLUE,
            secondary=ft.colors.BLUE_ACCENT
        )
    )
    page.update()

def main():
    # Inicializar rclpy antes de crear el nodo
    rclpy.init()

    try:
        # Crear una instancia del nodo ROS2
        node = MyNode()

        # Crear un hilo separado para ejecutar ros_spin
        ros_thread = threading.Thread(target=ros_spin, args=(node,))
        ros_thread.start()

        # Iniciar la interfaz Flet
        ft.app(target=interface)

        # Esperar a que el hilo de ROS termine (opcional)
        ros_thread.join()
    finally:
        # Cerrar rclpy correctamente
        rclpy.shutdown()

if __name__ == "__main__":
    main()
