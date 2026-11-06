# -*- coding: utf-8 -*-
"""ЛР3, шаг 2: выборка суставов по именам (тесты не требуют ROS)."""
import pytest

from pouk_bridge.joint_map import JointMap, JointMapError

NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper_joint']


def test_order_of_names_does_not_matter():
    m = JointMap(NAMES)
    shuffled = ['joint5', 'gripper_joint', 'joint1', 'joint3', 'joint2', 'joint4']
    values = [0.5, 0.6, 0.1, 0.3, 0.2, 0.4]
    assert m.to_stand(shuffled, values) == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]


def test_missing_joints_taken_from_current_or_rejected():
    m = JointMap(NAMES)
    cur = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert m.to_stand(['gripper_joint'], [0.3], current=cur) == [1.0, 2.0, 3.0, 4.0, 5.0, 0.3]
    with pytest.raises(JointMapError, match='нет суставов'):
        m.to_stand(['joint1'], [0.0])


def test_unknown_name_and_size_mismatch_rejected():
    m = JointMap(NAMES)
    with pytest.raises(JointMapError, match='неизвестный'):
        m.to_stand(NAMES[:5] + ['wrist_camera'], [0.0] * 6)
    with pytest.raises(JointMapError):
        m.to_stand(NAMES, [0.0] * 5)


def test_signs_and_offsets_roundtrip():
    m = JointMap(NAMES, signs=[1, -1, 1, 1, -1, 1], offsets=[0, 0.1, 0, 0, 0, -0.2])
    q_ros = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    q_stand = m.to_stand(NAMES, q_ros)
    assert q_stand == pytest.approx([0.1, -0.1, 0.3, 0.4, -0.5, 0.4])
    names, back = m.to_ros(q_stand)
    assert names == NAMES and back == pytest.approx(q_ros)


def test_bad_configuration_rejected():
    with pytest.raises(JointMapError):
        JointMap(['a', 'a'])
    with pytest.raises(JointMapError):
        JointMap(['a', 'b'], signs=[1, 2])
