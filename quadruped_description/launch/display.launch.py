import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Obtén las rutas correctas para los archivos
    urdf_path = os.path.join(get_package_share_directory('quadruped_description'),
                             'urdf','quadruped.urdf')
    
    rviz_config_path = os.path.join(get_package_share_directory('quadruped_description'),
                                    'rviz','urdf_config.rviz')
    
    # Lee el contenido del archivo URDF
    with open(urdf_path, 'r') as urdf_file:
        urdf_content = urdf_file.read()
    
    # Pasa el contenido del URDF como un parámetro en robot_state_publisher
    robot_description = ParameterValue(urdf_content, value_type=str)

    # Nodo robot_state_publisher
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output = 'screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    # Nodo joint_state_publisher
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    
    # Nodo joint_state_publisher_gui
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    
    # Nodo RViz
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output = 'screen',
        arguments=['-d', rviz_config_path]
    )
    
    # Retorna el LaunchDescription
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='true',
                                             description='Flag to enable joint_state_publisher_gui'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
