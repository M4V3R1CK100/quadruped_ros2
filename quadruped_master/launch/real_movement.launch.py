from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, ExecuteProcess



def generate_launch_description():

    joy = LaunchConfiguration('joy')
    image = LaunchConfiguration('image')
    
    joy_arg = DeclareLaunchArgument(
        'joy', default_value='False'
    )

    image_arg = DeclareLaunchArgument(
        'image', default_value='False'
    )   
    movement_node = Node(
        package='quadruped_master',
        executable='movement_node',
        output='screen'
    )
    dynamixel_communication_node = Node(
        package='quadruped_communication',
        executable='dynamixel_communication',
        output='screen'
    )    
    interface_node = Node(
        package='quadruped_app',
        executable='interface_node',
        output='screen',
        condition = UnlessCondition(joy)
    )  

    quadruped_visualizer_node = Node(
        package='quadruped_app',
        executable='quadruped_visualizer_node',
        output='screen',
        condition = UnlessCondition(joy)
    )

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        output='screen',
        condition = UnlessCondition(joy)
    )

    raw_to_bytes_node = Node(
        package='quadruped_app',
        executable='raw_to_bytes_node',
        output='screen',
        condition = UnlessCondition(joy), 
        parameters=[{'camera_topic': '/camera/camera/color/image_raw'}],
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen',
        condition = IfCondition(joy)
    )    
    
    joy_to_motion_node = Node(
        package='quadruped_master',
        executable='joy_to_motion_node',
        output='screen',
        condition = IfCondition(joy)   
    )

    image_node = Node(
        package='quadruped_app',
        executable='image_node',
        output='screen',
        condition = IfCondition(image)   
    )

    return LaunchDescription([
        joy_arg,
        image_arg, #  argumentos

        movement_node,
        # dynamixel_communication_node,
        realsense_node,
        interface_node,
        quadruped_visualizer_node,
        raw_to_bytes_node,
        joy_node,
        joy_to_motion_node,
        image_node,


    ])
