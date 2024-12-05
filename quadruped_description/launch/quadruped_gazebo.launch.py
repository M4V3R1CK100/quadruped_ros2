from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import Command


def generate_launch_description():
    # Configurar ruta para GAZEBO_MODEL_PATH
    # gazebo_model_path = os.path.join(
    #     get_package_share_directory('quadruped_gazebo'),
    #     'models'
    # )

    # Obtener la ruta absoluta del archivo .yaml
    # config_file = os.path.join(
    # get_package_share_directory('quadruped_description'),
    # 'config',
    # 'joint_controller.yaml'
    # )

    # Nodo para publicar el estado del robot
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

    # Incluir Gazebo desde su archivo de lanzamiento
    def get_gazebo_launch_description():
        gazebo_launch_path = os.path.join(
            get_package_share_directory('gazebo_ros'),
            'launch',
            'gazebo.launch.py'
        )
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path)
        )

    # Nodo para spawnear la entidad
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'quadruped'],
        output='screen'
    )

    # Procesos para cargar los controladores
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 
        '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=[
        'ros2', 'control', 'load_controller',
        '--set-state', 'active', 'joint_trajectory_controller'],
        output='screen'
    )

    # Ensamblar el launch description
    return LaunchDescription([
        # # Establecer GAZEBO_MODEL_PATH dinámicamente
        # SetEnvironmentVariable(
        #     name='GAZEBO_MODEL_PATH',
        #     value=gazebo_model_path
        # ),
        # Lanzar Gazebo
        get_gazebo_launch_description(),
        # Publicar robot_description
        get_robot_state_publisher_node(),
        # Spawnear el robot
        spawn_entity_node,
        # Cargar controladores después de spawnear
        # RegisterEventHandler(
        #     OnProcessExit(
        #         target_action=spawn_entity_node,
        #         on_exit=[
        #             load_joint_state_broadcaster,
        #             load_joint_trajectory_controller
        #         ]
        #     )
        # )
    ])
