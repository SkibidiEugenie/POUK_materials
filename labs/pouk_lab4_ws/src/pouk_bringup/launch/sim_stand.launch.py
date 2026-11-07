# -*- coding: utf-8 -*-
"""Виртуальный стенд на одной машине: модель МК (pouk_link.sim_mcu) + мост + robot_state_publisher.

    ros2 launch pouk_bringup sim_stand.launch.py
    ros2 launch pouk_bringup sim_stand.launch.py stuck:=2@15     # сустав 2 заклинит на 15-й секунде
Мост, координатор и MoveIt работают с моделью так же, как со стендом (пособие, §7.6.4).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _sim_process(context):
    port = LaunchConfiguration('sim_port').perform(context)
    cmd = ['ros2', 'run', 'pouk_link', 'sim_mcu', '--port', port,
           '--wd-ms', LaunchConfiguration('wd_ms').perform(context)]
    stuck = LaunchConfiguration('stuck').perform(context)
    if stuck:
        cmd += ['--stuck', stuck]
    return [ExecuteProcess(cmd=cmd, output='screen', name='sim_mcu')]


def generate_launch_description():
    bringup = get_package_share_directory('pouk_bringup')
    xacro_file = os.path.join(get_package_share_directory('pouk_description'), 'urdf', 'pouk_arm.urdf.xacro')
    return LaunchDescription([
        DeclareLaunchArgument('sim_port', default_value='5760'),
        DeclareLaunchArgument('wd_ms', default_value='60', description='тайм-аут сторожевого таймера модели'),
        DeclareLaunchArgument('stuck', default_value='', description='J@T: сустав J заклинит в момент T, с'),
        DeclareLaunchArgument('rsp', default_value='true', description='запускать robot_state_publisher'),
        OpaqueFunction(function=_sim_process),
        Node(package='pouk_bridge', executable='bridge', name='stand_bridge', output='screen',
             parameters=[os.path.join(bringup, 'config', 'sim.yaml'),
                         {'port': ['socket://127.0.0.1:', LaunchConfiguration('sim_port')]}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
                         'publish_frequency': 100.0}],
             condition=IfCondition(LaunchConfiguration('rsp'))),
    ])
