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

    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen'
    )    
    
    joy_to_motion_node = Node(
        package='quadruped_master',
        executable='joy_to_motion_node',
        output='screen'
    )    
    
    # Retorna el LaunchDescription
    return launch.LaunchDescription([
        movement_node,
        joy_node,
        joy_to_motion_node
    ])
