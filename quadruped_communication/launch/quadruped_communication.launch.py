from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declarar argumentos
    usb_port0_arg = DeclareLaunchArgument(
        'usb_port0', default_value='/dev/ttyUSB0', description='Puerto USB 0'
    )
    usb_port1_arg = DeclareLaunchArgument(
        'usb_port1', default_value='/dev/ttyUSB1', description='Puerto USB 1'
    )
    dxl_baud_rate_arg = DeclareLaunchArgument(
        'dxl_baud_rate', default_value='57600', description='Velocidad de comunicación para Dynamixel'
    )

    # Nodo
    quadruped_communication_node = Node(
        package='quadruped_communication',
        executable='u2d2_communication.py',
        name='quadruped_communication',
        output='screen',
        parameters=[
            {
                'usb_port0': LaunchConfiguration('usb_port0'),
                'usb_port1': LaunchConfiguration('usb_port1'),
                'dxl_baud_rate': LaunchConfiguration('dxl_baud_rate'),
            }
        ]
    )

    # Descripción del lanzamiento
    return LaunchDescription([
        usb_port0_arg,
        usb_port1_arg,
        dxl_baud_rate_arg,
        quadruped_communication_node
    ])
