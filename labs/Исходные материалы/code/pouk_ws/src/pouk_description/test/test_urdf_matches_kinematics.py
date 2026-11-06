# -*- coding: utf-8 -*-
"""URDF и модуль кинематики описывают один и тот же манипулятор.

Если длины или знаки в URDF разойдутся с pouk_link.kinematics, MoveIt и RViz будут
показывать одно, а координатор ЛР3 — делать другое. Тест ловит это до стенда.
"""
import math
import os
import random

import numpy as np
import pytest
import xacro

from pouk_link import kinematics as K

HERE = os.path.dirname(os.path.abspath(__file__))
XACRO = os.path.join(HERE, '..', 'urdf', 'pouk_arm.urdf.xacro')
ARM = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper_joint']


def load_model():
    from xml.etree import ElementTree as ET
    root = ET.fromstring(xacro.process_file(XACRO).toxml())
    joints = {}
    for j in root.findall('joint'):
        origin = j.find('origin')
        xyz = [float(v) for v in origin.get('xyz').split()] if origin is not None else [0, 0, 0]
        axis = j.find('axis')
        lim = j.find('limit')
        joints[j.get('name')] = dict(
            type=j.get('type'), parent=j.find('parent').get('link'), child=j.find('child').get('link'),
            xyz=np.array(xyz), axis=np.array([float(v) for v in axis.get('xyz').split()]) if axis is not None else None,
            lower=float(lim.get('lower')) if lim is not None else None,
            upper=float(lim.get('upper')) if lim is not None else None)
    return joints


def rot(axis, a):
    x, y, z = axis / np.linalg.norm(axis)
    c, s, C = math.cos(a), math.sin(a), 1 - math.cos(a)
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def frame_of(joints, link, q):
    """Положение и ориентация звена link в base_link при углах q (словарь имя -> угол)."""
    chain = []
    while link != 'base_link':
        j = next(v for v in joints.items() if v[1]['child'] == link)
        chain.append(j)
        link = j[1]['parent']
    R, p = np.eye(3), np.zeros(3)
    for name, j in reversed(chain):
        p = p + R @ j['xyz']
        if j['type'] == 'revolute':
            R = R @ rot(j['axis'], q.get(name, 0.0))
    return R, p


def test_limits_equal_firmware_limits():
    joints = load_model()
    for i, name in enumerate(ARM):
        assert joints[name]['lower'] == pytest.approx(K.JOINT_MIN[i], abs=1e-9)
        assert joints[name]['upper'] == pytest.approx(K.JOINT_MAX[i], abs=1e-9)


def test_tool0_matches_fk_and_ik():
    joints = load_model()
    rng = random.Random(5)
    checked = 0
    while checked < 200:
        x, y, z = rng.uniform(-0.15, 0.15), rng.uniform(-0.15, 0.15), rng.uniform(-0.05, 0.12)
        r = K.ik(x, y, z)
        if not r.ok:
            continue
        q = dict(zip(ARM, K.with_gripper(r.q)))
        R, p = frame_of(joints, 'tool0', q)
        assert p == pytest.approx([x, y, z], abs=1e-6)        # ОЗК -> URDF возвращает цель
        assert p == pytest.approx(K.fk(r.q), abs=1e-6)        # и совпадает с ПЗК модуля
        assert R[:, 2] == pytest.approx([0, 0, -1], abs=1e-6)  # схват смотрит вниз
        checked += 1


def test_zero_pose_is_vertical():
    joints = load_model()
    _, p = frame_of(joints, 'tool0', {})
    assert p == pytest.approx([0, 0, K.L1 + K.L2 + K.L3 + K.L4], abs=1e-9)
