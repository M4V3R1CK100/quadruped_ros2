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
    communication_copplelia_node = Node(
        package='quadruped_master',
        executable='communication_copplelia_node',
        output='screen'
    )    
    interface_node = Node(
        package='quadruped_app',
        executable='interface_node',
        output='screen'
    )  
    
    server_copplelia_node = Node(
        package='quadruped_vrep',
        executable='communication',
        output='screen'
    )
    # Retorna el LaunchDescription
    return launch.LaunchDescription([
        # movement_node,
        communication_copplelia_node,
        interface_node,
        server_copplelia_node
    ])
