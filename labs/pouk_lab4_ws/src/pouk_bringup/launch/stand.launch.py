# -*- coding: utf-8 -*-
"""Бортовая часть на одноплатнике NanoPi: только мост к микроконтроллеру.

    ros2 launch pouk_bringup stand.launch.py                      # порт /dev/ttyS1 из config/stand.yaml
    ros2 launch pouk_bringup stand.launch.py port:=/dev/ttyACM0   # OpenCM по USB (отладка на ПК)
Описание робота, координатор, RViz и MoveIt запускаются на ПК (lab3.launch.py, lab4a.launch.py):
одноплатнику с 512 МБ памяти оставлено то, что обязано быть рядом с железом.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('pouk_bringup'), 'config', 'stand.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyS1'),
        DeclareLaunchArgument('log_prefix', default_value=''),
        Node(package='pouk_bridge', executable='bridge', name='stand_bridge', output='screen',
             parameters=[config, {'port': LaunchConfiguration('port'),
                                  'log_prefix': LaunchConfiguration('log_prefix')}]),
    ])
