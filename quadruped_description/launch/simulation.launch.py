import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
import xacro


def generate_launch_description():

    pkg_name = 'quadruped_description'  # the package name

    pkg_share = get_package_share_directory(pkg_name)

    urdf_path = 'urdf/quadruped.urdf.xacro'

    # extracting the robot deffinition from the xacro file
    xacro_file = os.path.join(pkg_share, urdf_path)
    robot_description_content = xacro.process_file(xacro_file).toxml()

    # add the path to the model file to  gazebo
    models_path = os.path.join(get_package_share_directory(pkg_name), 'meshes')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path = os.environ['GAZEBO_MODEL_PATH'] + ':' + models_path
    else:
        model_path = models_path

    world_path = os.path.join(pkg_share, 'worlds', 'world_test_wq.world')

    # robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )
    # Gazebo launch file
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        )
        # launch_arguments={'world': world_path, 'verbose': 'true'}.items()
    )
    # entity spawn node (to spawn the robot from the /robot_description topic)
    node_spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                             arguments=['-topic', 'robot_description',
                                        '-entity', 'quadruped'],
                             output='screen')
    
    # spawning the joint broadcaster
    # Procesos para cargar los controladores
    spawn_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 
        '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    # JOINT TRAJECTORY CONTROLLER
    spawn_controller = ExecuteProcess(
        cmd=[
        'ros2', 'control', 'load_controller',
        '--set-state', 'active', 'joint_trajectory_controller'],
        output='screen'
    )


    #FORWARD COMMAND CONTROLLER
    # spawn_controller = ExecuteProcess(
    #     cmd=[
    #         'ros2', 'control', 'load_controller',
    #         '--set-state', 'active', 'position_controller'],
    #     output='screen'
    # )


    # Run the nodes
    return LaunchDescription([
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path),
        node_robot_state_publisher,
        launch_gazebo,
        node_spawn_entity,
        # node_rviz,
        # Cargar controladores después de spawnear
        RegisterEventHandler(
            OnProcessExit(
                target_action=node_spawn_entity,
                on_exit=[
                    TimerAction(
                        period=5.0,  # Delay en segundos (ajusta este valor)
                        actions=[
                            spawn_broadcaster,
                            spawn_controller
                        ]
                    )
                ]
            )
        )
    ])
