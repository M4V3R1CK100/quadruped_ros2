from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    # Path al archivo URDF/XACRO
    urdf_path = os.path.join(get_package_share_directory('quadruped_description'), 'urdf', 'quadruped.xacro')

    # Path al launch file de Gazebo
    gazebo_launch_path = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')

    return LaunchDescription([
        # Publicar el modelo URDF/XACRO como 'robot_description'
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', urdf_path])}]
        ),

        # Incluir Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path)
        ),

        # Spawnear el modelo en Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'quadruped'],
            output='screen'
        ),
    ])
