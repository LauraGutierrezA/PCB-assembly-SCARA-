"""
twin.launch.py - Fase 2: el gemelo en RViz siguiendo al robot (real o falso).

  Robot real (con el micro-ROS agent ya corriendo):
      ros2 launch scara_twin twin.launch.py

  Sin hardware (robot falso que se mueve solo):
      ros2 launch scara_twin twin.launch.py fake:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc = get_package_share_directory('scara_description')
    xacro_file = os.path.join(desc, 'urdf', 'scara.urdf.xacro')
    rviz_file = os.path.join(desc, 'rviz', 'scara.rviz')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('fake', default_value='false',
                              description='true: robot falso en vez del ESP32 real'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Abrir RViz'),
        DeclareLaunchArgument('smoothing', default_value='0.4',
                              description='Suavizado del gemelo (0..1, 1 = sin suavizar)'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='scara_twin',
            executable='bridge',
            parameters=[{'smoothing': ParameterValue(LaunchConfiguration('smoothing'),
                                                     value_type=float)}],
            output='screen',
        ),
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
