"""
display.launch.py - Fase 1: ver el modelo del SCARA en RViz con sliders.

  ros2 launch scara_description display.launch.py

No necesita el robot ni micro-ROS: joint_state_publisher_gui publica
/joint_states con tres sliders (joint1, joint2 en rad; joint3 en m).
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
    pkg = get_package_share_directory('scara_description')
    xacro_file = os.path.join(pkg, 'urdf', 'scara.urdf.xacro')
    rviz_file = os.path.join(pkg, 'rviz', 'scara.rviz')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Abrir los sliders de joint_state_publisher_gui'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Abrir RViz'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('gui')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_file],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
