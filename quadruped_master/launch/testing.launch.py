from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, ExecuteProcess



def generate_launch_description():

    dynamixel_communication_node = Node(
        package='quadruped_communication',
        executable='dynamixel_communication',
        output='screen'
    )    

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        output='screen',
    )



    return LaunchDescription([
        dynamixel_communication_node,
        realsense_node,


    ])
