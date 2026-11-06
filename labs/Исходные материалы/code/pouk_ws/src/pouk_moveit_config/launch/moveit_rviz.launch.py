# -*- coding: utf-8 -*-
"""RViz с панелью планирования MoveIt для стенда ПОУК."""
import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory('pouk_description'), 'urdf', 'pouk_arm.urdf.xacro')
    config = (MoveItConfigsBuilder('pouk_arm', package_name='pouk_moveit_config')
              .robot_description(file_path=xacro_file)
              .robot_description_semantic(file_path='config/pouk_arm.srdf')
              .robot_description_kinematics(file_path='config/kinematics.yaml')
              .joint_limits(file_path='config/joint_limits.yaml')
              .planning_pipelines(pipelines=['ompl'])
              .to_moveit_configs())
    return generate_moveit_rviz_launch(config)
