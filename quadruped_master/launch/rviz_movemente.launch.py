import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit
import xacro

def generate_launch_description():
    # Obtén las rutas correctas para los archivos
    
    rviz_config_path = os.path.join(get_package_share_directory('quadruped_description'),
                                    'rviz','urdf_config.rviz')
    
    
    pkg_name = 'quadruped_description'  # the package name

    pkg_share = get_package_share_directory(pkg_name)

    urdf_path = 'urdf/quadruped.urdf.xacro'

    # extracting the robot deffinition from the xacro file
    xacro_file = os.path.join(pkg_share, urdf_path)
    robot_description = xacro.process_file(xacro_file).toxml()
    
  
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

    custom_joint_state_node = launch_ros.actions.Node(
        package='quadruped_master',  # Reemplaza con el nombre de tu paquete
        executable='movement_node',  # Nombre del ejecutable de tu script
        name='movement_node',
        output='screen',
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
        custom_joint_state_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        rviz_node
    ])
