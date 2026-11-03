"""
twin_gazebo.launch.py - Fase 3: el gemelo en Gazebo (y RViz) siguiendo al robot.

  Robot real (con el micro-ROS agent ya corriendo):
      ros2 launch scara_twin twin_gazebo.launch.py

  Sin hardware (robot falso que se mueve solo):
      ros2 launch scara_twin twin_gazebo.launch.py fake:=true

Orden: robot_state_publisher -> Gazebo -> spawn del modelo -> (al terminar el spawn)
controlador de posicion. El puente manda las mismas posiciones a RViz (/joint_states) y a
Gazebo (/forward_position_controller/commands).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc = get_package_share_directory('scara_description')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    xacro_file = os.path.join(desc, 'urdf', 'scara.urdf.xacro')
    rviz_file = os.path.join(desc, 'rviz', 'scara.rviz')

    # use_gz:=true agrega world fijo, ros2_control y el plugin de Gazebo al modelo.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_gz:=true']), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            # -r: arrancar la simulacion en marcha (sin esto empieza en pausa)
            'gz_args': PythonExpression(["'-r ' + '", LaunchConfiguration('world'), "'"]),
        }.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'scara'],
        output='screen',
    )

    # El controller_manager lo crea el plugin dentro de Gazebo cuando entra el modelo.
    position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '30'],
        output='screen',
    )

    bridge = Node(
        package='scara_twin',
        executable='bridge',
        parameters=[{
            'publish_gz_commands': True,
            'smoothing': ParameterValue(LaunchConfiguration('smoothing'), value_type=float),
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('fake', default_value='false',
                              description='true: robot falso en vez del ESP32 real'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Abrir RViz ademas de Gazebo'),
        DeclareLaunchArgument('world', default_value='empty.sdf',
                              description='Mundo de Gazebo'),
        DeclareLaunchArgument('smoothing', default_value='0.4',
                              description='Suavizado del gemelo (0..1, 1 = sin suavizar)'),

        robot_state_publisher,
        gazebo,
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn,
                                           on_exit=[position_controller])),
        bridge,
        Node(
            package='scara_twin',
            executable='fake_scara',
            condition=IfCondition(LaunchConfiguration('fake')),
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_file],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
