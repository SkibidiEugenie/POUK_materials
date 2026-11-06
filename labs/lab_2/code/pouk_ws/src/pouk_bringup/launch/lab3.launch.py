# -*- coding: utf-8 -*-
"""ЛР3, сторона ПК: описание робота, координатор, RViz. Одна команда на сценарий.

    ros2 launch pouk_bringup lab3.launch.py sim:=true     # всё на ПК, стенд — модель
    ros2 launch pouk_bringup lab3.launch.py               # стенд: на NanoPi уже запущен stand.launch.py
Обе машины — в одном домене: одинаковый ROS_DOMAIN_ID (и одна подсеть).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = get_package_share_directory('pouk_bringup')
    description = get_package_share_directory('pouk_description')
    xacro_file = os.path.join(description, 'urdf', 'pouk_arm.urdf.xacro')
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false', description='виртуальный стенд вместо NanoPi'),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'sim_stand.launch.py')),
            launch_arguments={'rsp': 'false'}.items(),
            condition=IfCondition(LaunchConfiguration('sim'))),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
                         'publish_frequency': 100.0}]),
        Node(package='pouk_coordinator', executable='coordinator', name='coordinator', output='screen',
             parameters=[os.path.join(bringup, 'config', 'coordinator.yaml')]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(description, 'rviz', 'display.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
