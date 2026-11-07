# -*- coding: utf-8 -*-
"""Описания запуска собираются: пакеты, файлы конфигурации и аргументы находятся."""
import importlib.util
import os

import pytest
from ament_index_python.packages import get_package_share_directory

LAUNCH = ['sim_stand.launch.py', 'stand.launch.py', 'lab3.launch.py', 'lab4a.launch.py']


def load(path):
    spec = importlib.util.spec_from_file_location(os.path.basename(path).replace('.', '_'), path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.mark.parametrize('name', LAUNCH)
def test_launch_description_builds(name):
    path = os.path.join(os.path.dirname(__file__), '..', 'launch', name)
    ld = load(path).generate_launch_description()
    assert ld.entities


def test_moveit_config_loads():
    path = os.path.join(get_package_share_directory('pouk_moveit_config'), 'launch', 'move_group.launch.py')
    cfg = load(path).moveit_config().to_dict()
    assert cfg['robot_description_kinematics']['arm']['position_only_ik'] is True
    limits = cfg['robot_description_planning']['joint_limits']
    assert all(limits['joint%d' % i]['has_acceleration_limits'] for i in range(1, 6))
    ctrl = cfg['moveit_simple_controller_manager']['arm_controller']
    assert ctrl['action_ns'] == 'follow_joint_trajectory' and len(ctrl['joints']) == 5
