# -*- coding: utf-8 -*-
"""ЛР4-А, шаг 3: интерполяция по содержимому сообщения (тесты не требуют ROS)."""
import pytest

from pouk_bridge.trajectory import CUBIC, LINEAR, QUINTIC, Trajectory, TrajectoryError


def num_derivative(tr, t, h=1e-6):
    return [(a - b) / (2 * h) for a, b in zip(tr.sample(t + h)[0], tr.sample(t - h)[0])]


def test_kind_chosen_by_content():
    assert Trajectory([0, 1], [[0], [1]]).kind == LINEAR
    assert Trajectory([0, 1], [[0], [1]], [[0], [0]]).kind == CUBIC
    assert Trajectory([0, 1], [[0], [1]], [[0], [0]], [[0], [0]]).kind == QUINTIC


def test_linear_passes_points_with_constant_velocity():
    tr = Trajectory([0.0, 2.0], [[0.0, 1.0], [1.0, -1.0]])
    q, v = tr.sample(0.5)
    assert q == pytest.approx([0.25, 0.5]) and v == pytest.approx([0.5, -1.0])


@pytest.mark.parametrize('with_acc', [False, True])
def test_polynomials_meet_boundary_conditions(with_acc):
    times = [0.0, 0.8, 2.0]
    pos = [[0.0], [0.5], [0.2]]
    vel = [[0.0], [0.4], [0.0]]
    acc = [[0.0], [-0.3], [0.0]] if with_acc else None
    tr = Trajectory(times, pos, vel, acc)
    for k, t in enumerate(times):
        eps = 1e-9 if 0 < k < len(times) - 1 else 0.0
        for side in ((-eps, eps) if eps else (0.0,)):
            q, v = tr.sample(t + side)
            assert q[0] == pytest.approx(pos[k][0], abs=1e-6)
            assert v[0] == pytest.approx(vel[k][0], abs=1e-6)
    # скорость, которую возвращает sample, — производная положения
    for t in (0.3, 1.1, 1.7):
        assert tr.sample(t)[1] == pytest.approx(num_derivative(tr, t), abs=1e-5)


def test_start_segment_from_actual_state():
    tr = Trajectory([1.0], [[1.0]], start=[0.0])
    assert tr.duration == 1.0
    assert tr.sample(0.0)[0] == [0.0] and tr.sample(0.5)[0] == pytest.approx([0.5])
    with pytest.raises(TrajectoryError):
        Trajectory([1.0], [[1.0]])                       # начало неизвестно


def test_hold_after_end():
    tr = Trajectory([0.0, 1.0], [[0.0], [1.0]], [[0.0], [0.0]])
    assert tr.sample(5.0) == ([1.0], [0.0])


@pytest.mark.parametrize('times,pos,vel', [
    ([], [], None),
    ([0.0, 0.0], [[0.0], [1.0]], None),                  # время не возрастает
    ([0.0, 1.0], [[0.0], [1.0, 2.0]], None),             # разное число суставов
    ([0.0, 1.0], [[0.0], [float('nan')]], None),         # не число
    ([0.0, 1.0], [[0.0], [1.0]], [[0.0], []]),           # скорости не во всех точках
])
def test_invalid_trajectories_rejected(times, pos, vel):
    with pytest.raises(TrajectoryError):
        Trajectory(times, pos, vel)
