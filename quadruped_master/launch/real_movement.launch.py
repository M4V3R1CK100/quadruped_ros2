from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    
    movement_node = Node(
        package='quadruped_master',
        executable='movement_node',
        output='screen'
    )
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        output='screen'
    )
    visualizer_node = Node(
        package='quadruped_app',
        executable='quadruped_visualizer_node',
        output='screen'
    )
    raw_to_bytes_node = Node(
        package='quadruped_app',
        executable='raw_to_bytes_node',
        output='screen'
    )
    # Retorna el LaunchDescription
    return launch.LaunchDescription([
        movement_node,
        realsense_node,
        visualizer_node, 
        raw_to_bytes_node

    ])
