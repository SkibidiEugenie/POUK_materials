# -*- coding: utf-8 -*-
"""ЛР4-А, сторона ПК: MoveIt (move_group + RViz) и описание робота; исполнение — мостом стенда.

    ros2 launch pouk_bringup lab4a.launch.py sim:=true    # всё на ПК, стенд — модель
    ros2 launch pouk_bringup lab4a.launch.py              # стенд: на NanoPi запущен stand.launch.py
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
    moveit = get_package_share_directory('pouk_moveit_config')
    xacro_file = os.path.join(get_package_share_directory('pouk_description'), 'urdf', 'pouk_arm.urdf.xacro')
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false', description='виртуальный стенд вместо NanoPi'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('stuck', default_value='', description='для sim: J@T — заклинивание сустава'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'sim_stand.launch.py')),
            launch_arguments={'rsp': 'false', 'stuck': LaunchConfiguration('stuck')}.items(),
            condition=IfCondition(LaunchConfiguration('sim'))),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
                         'publish_frequency': 100.0}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(moveit, 'launch', 'move_group.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(moveit, 'launch', 'moveit_rviz.launch.py')),
                                 condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
