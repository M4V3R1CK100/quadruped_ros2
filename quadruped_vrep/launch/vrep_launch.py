import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='quadruped_vrep',  # Nombre del paquete
            executable='vrep_communication',  # Nombre del ejecutable (tipo del nodo)
            name='vrep_communication',  # Nombre del nodo
            output='screen'  # Muestra la salida en la terminal
        ),
    ])
