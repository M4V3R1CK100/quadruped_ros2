from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    
    gazebo_model_path = os.path.join(
        get_package_share_directory('quadruped_gazebo'),
        'models'
    )

    def get_robot_state_publisher_node():
        urdf_path = os.path.join(
            get_package_share_directory('quadruped_description'),
            'urdf',
            'quadruped.xacro'
        )
        return Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', urdf_path])}]
        )

    def get_gazebo_launch_description():
        gazebo_launch_path = os.path.join(
            get_package_share_directory('gazebo_ros'),
            'launch',
            'gazebo.launch.py'
        )
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path)
        )

    def get_spawn_entity_node():
        return Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'quadruped'],
            output='screen'
        )
    load_joint_state_controller = ExecuteProcess
    

    # Ensamblar las funciones
    return LaunchDescription([
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=gazebo_model_path),
        get_robot_state_publisher_node(),
        get_gazebo_launch_description(),
        get_spawn_entity_node(),
    ])
