# -*- coding: utf-8 -*-
"""Модель стенда в RViz: robot_state_publisher + (по желанию) ползунки суставов.

    ros2 launch pouk_description display.launch.py            # ползунки joint_state_publisher_gui
    ros2 launch pouk_description display.launch.py gui:=false # углы приходят от моста (/joint_states)
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
    share = get_package_share_directory('pouk_description')
    xacro_file = os.path.join(share, 'urdf', 'pouk_arm.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='ползунки суставов'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(share, 'rviz', 'display.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
