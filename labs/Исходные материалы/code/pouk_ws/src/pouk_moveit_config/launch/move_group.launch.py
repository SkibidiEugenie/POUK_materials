# -*- coding: utf-8 -*-
"""move_group для стенда ПОУК: планирование OMPL, ОЗК KDL (только положение), исполнение — мостом.

    ros2 launch pouk_moveit_config move_group.launch.py
Обычно запускается из pouk_bringup/lab4a.launch.py вместе с RViz и (или) виртуальным стендом.
"""
import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def moveit_config():
    xacro_file = os.path.join(get_package_share_directory('pouk_description'), 'urdf', 'pouk_arm.urdf.xacro')
    return (MoveItConfigsBuilder('pouk_arm', package_name='pouk_moveit_config')
            .robot_description(file_path=xacro_file)
            .robot_description_semantic(file_path='config/pouk_arm.srdf')
            .robot_description_kinematics(file_path='config/kinematics.yaml')
            .joint_limits(file_path='config/joint_limits.yaml')
            .trajectory_execution(file_path='config/moveit_controllers.yaml')
            .planning_pipelines(pipelines=['ompl'])
            .to_moveit_configs())


def generate_launch_description():
    return generate_move_group_launch(moveit_config())
