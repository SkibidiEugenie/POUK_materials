# -*- coding: utf-8 -*-
"""ЛР4-А, шаг 4: контракт исполнителя траектории на модели привода (тесты не требуют ROS)."""
import pytest

from pouk_bridge import executor as E

NAMES = ['j1', 'j2']
LIMITS = {'j1': (-1.5, 1.5), 'j2': (-1.5, 1.5)}


class FakeArm(object):
    """Привод с ограничением скорости и транспортной задержкой одного периода."""

    def __init__(self, q, vmax=2.0, stuck=None):
        self.q = dict(q)
        self.goal = dict(q)
        self.vmax = vmax
        self.stuck = stuck or {}

    def step(self, dt, t):
        for n in self.q:
            if n in self.stuck and t >= self.stuck[n]:
                continue
            e = self.goal[n] - self.q[n]
            self.q[n] += max(-self.vmax * dt, min(self.vmax * dt, e))


def run(exe, arm, dt=0.01, t_max=10.0):
    t = 0.0
    while t < t_max:
        st = exe.step(t, dict(arm.q))
        if st.status == E.DONE:
            return st, t
        arm.goal.update(zip(exe.names, st.desired))
        arm.step(dt, t)
        t += dt
    raise AssertionError('исполнитель не завершился за %.1f с' % t_max)


def start(exe, arm, times, pos, vel=None, **kw):
    vel = vel or [[] for _ in times]
    return exe.start(NAMES, times, pos, vel, [[] for _ in times], dict(arm.q), 0.0,
                     known_names=NAMES, limits=LIMITS, **kw)


def test_success_and_goal_reached():
    exe, arm = E.TrajectoryExecutor(), FakeArm({'j1': 0.0, 'j2': 0.0})
    assert start(exe, arm, [0.0, 1.0, 2.0], [[0.0, 0.0], [0.4, -0.2], [0.8, -0.4]]) is None
    st, t = run(exe, arm)
    assert st.code == E.SUCCESSFUL and 2.0 < t < 2.5
    assert arm.q['j1'] == pytest.approx(0.8, abs=0.05)


def test_stuck_joint_violates_path_tolerance_during_motion():
    exe = E.TrajectoryExecutor(path_tolerance=0.1)
    arm = FakeArm({'j1': 0.0, 'j2': 0.0}, stuck={'j2': 0.5})
    start(exe, arm, [0.0, 2.0], [[0.0, 0.0], [1.0, 1.0]])
    st, t = run(exe, arm)
    assert st.code == E.PATH_TOLERANCE_VIOLATED and 'j2' in st.message
    assert t < 2.0                                       # отказ в движении, а не после конца


def test_slow_drive_violates_goal_time_tolerance():
    exe = E.TrajectoryExecutor(path_tolerance=-1, goal_time_tolerance=0.3)
    arm = FakeArm({'j1': 0.0, 'j2': 0.0}, vmax=0.2)
    start(exe, arm, [0.0, 1.0], [[0.0, 0.0], [1.0, 0.0]], path_tol=[('j1', -1.0), ('j2', -1.0)])
    st, t = run(exe, arm)
    assert st.code == E.GOAL_TOLERANCE_VIOLATED and 1.3 <= t < 1.4


def test_tolerance_message_semantics():
    tol = E.Tolerances(NAMES, 0.1).apply([('j1', 0.0), ('j2', -1.0)])
    assert tol.value == {'j1': 0.1, 'j2': None}          # 0 — по умолчанию, < 0 — не проверять
    assert tol.exceeded([0.05, 5.0]) is None and tol.exceeded([0.2, 0.0]) == 'j1'


@pytest.mark.parametrize('names,times,pos,code', [
    (['j1', 'j9'], [0.0, 1.0], [[0, 0], [1, 1]], E.INVALID_JOINTS),
    (['j1', 'j1'], [0.0, 1.0], [[0, 0], [1, 1]], E.INVALID_JOINTS),
    (NAMES, [1.0, 0.5], [[0, 0], [1, 1]], E.INVALID_GOAL),         # время убывает
    (NAMES, [0.0, 1.0], [[0, 0], [2.0, 0]], E.INVALID_GOAL),       # за пределом сустава
    (NAMES, [0.0, 1.0], [[0.5, 0], [0.6, 0]], E.INVALID_GOAL),     # начало далеко от факта
])
def test_invalid_goals_get_typed_codes(names, times, pos, code):
    exe = E.TrajectoryExecutor(start_tolerance=0.1)
    fail = exe.start(names, times, pos, [[] for _ in times], [[] for _ in times],
                     {'j1': 0.0, 'j2': 0.0}, 0.0, known_names=NAMES, limits=LIMITS)
    assert fail is not None and fail.code == code and fail.message


def test_first_point_in_future_starts_from_actual_state():
    exe, arm = E.TrajectoryExecutor(), FakeArm({'j1': 0.3, 'j2': 0.0})
    assert start(exe, arm, [1.0], [[0.0, 0.0]]) is None   # точка через 1 с: доводка от факта
    assert exe.step(0.0, dict(arm.q)).desired == pytest.approx([0.3, 0.0])
    st, _ = run(exe, arm)
    assert st.code == E.SUCCESSFUL
