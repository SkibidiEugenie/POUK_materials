# -*- coding: utf-8 -*-
"""Исполнитель траектории: контракт FollowJointTrajectory без ROS (пособие, §4.5.4).

Предположения (обязанности клиента): суставы известны исполнителю; время точек
возрастает; траектория начинается вблизи фактического состояния; углы в пределах.
Гарантии (обязанности исполнителя): пока отклонение каждого сустава не превышает
допуска по пути, движение идёт по интерполированному эталону; к моменту
t_кон + допуск_по_времени все суставы в допуске по цели; иначе исполнение
НЕМЕДЛЕННО прекращается, привод удерживает текущее положение, клиент получает
типизированный код отказа.

Ядро пошаговое: step(now, actual) вызывается с постоянным периодом и возвращает
эталон для отправки на стенд и исход. Время — секунды монотонных часов.
"""
from collections import namedtuple

from .trajectory import Trajectory, TrajectoryError

# коды результата — те же числа, что в control_msgs/action/FollowJointTrajectory
SUCCESSFUL = 0
INVALID_GOAL = -1
INVALID_JOINTS = -2
OLD_HEADER_TIMESTAMP = -3
PATH_TOLERANCE_VIOLATED = -4
GOAL_TOLERANCE_VIOLATED = -5

RUNNING = 'RUNNING'
DONE = 'DONE'

Step = namedtuple('Step', 'status code message desired desired_vel error')


class Tolerances(object):
    """Допуски по суставам. Правило сообщения JointTolerance: > 0 — значение,
    0 — значение по умолчанию исполнителя, < 0 — не проверять."""

    def __init__(self, names, default):
        self.names = list(names)
        self.value = {n: (default if default > 0 else None) for n in self.names}

    def apply(self, overrides):
        """overrides — список (имя, значение) из цели. Неизвестное имя — ошибка."""
        for name, v in overrides:
            if name not in self.value:
                raise KeyError(name)
            if v > 0:
                self.value[name] = v
            elif v < 0:
                self.value[name] = None                 # проверка отключена
        return self

    def exceeded(self, errors):
        """Первый сустав, у которого |ошибка| больше допуска, или None."""
        for name, e in zip(self.names, errors):
            tol = self.value[name]
            if tol is not None and abs(e) > tol:
                return name
        return None


class TrajectoryExecutor(object):
    def __init__(self, path_tolerance=0.15, goal_tolerance=0.05, goal_time_tolerance=1.0,
                 start_tolerance=0.10):
        self.default_path = path_tolerance
        self.default_goal = goal_tolerance
        self.default_goal_time = goal_time_tolerance
        self.start_tolerance = start_tolerance
        self.traj = None
        self.names = []

    # ---------------------------------------------------------------- начало
    def start(self, names, times, positions, velocities, accelerations, actual, t_start,
              path_tol=(), goal_tol=(), goal_time_tol=0.0, known_names=None, limits=None):
        """Проверить цель и подготовить исполнение.

        actual — словарь «имя -> фактический угол» для всех суставов траектории.
        Возвращает None, если цель принята, иначе Step(DONE, код, сообщение).
        """
        self.traj = None
        names = list(names)
        known = set(known_names) if known_names is not None else set(actual)
        if not names or len(set(names)) != len(names):
            return self._fail(INVALID_JOINTS, 'пустой или повторяющийся список суставов')
        unknown = [n for n in names if n not in known]
        if unknown:
            return self._fail(INVALID_JOINTS, 'суставы %s не принадлежат исполнителю' % unknown)
        start = [actual[n] for n in names]
        try:
            traj = Trajectory(times, positions, velocities, accelerations, start=start)
        except TrajectoryError as e:
            return self._fail(INVALID_GOAL, str(e))
        if limits is not None:
            for k, point in enumerate(traj.q):
                for n, v in zip(names, point):
                    lo, hi = limits[n]
                    if not lo <= v <= hi:
                        return self._fail(INVALID_GOAL, 'точка %d: %s = %.3f вне пределов [%.3f; %.3f]'
                                          % (k, n, v, lo, hi))
        try:
            self.path = Tolerances(names, self.default_path).apply(path_tol)
            self.goal = Tolerances(names, self.default_goal).apply(goal_tol)
        except KeyError as e:
            return self._fail(INVALID_JOINTS, 'допуск задан для чужого сустава %s' % e)
        self.goal_time = goal_time_tol if goal_time_tol > 0 else self.default_goal_time
        # >>> STUDENT ЛР4А-шаг4: проверка начала. Если первая точка задана на момент 0, фактическое
        # >>> состояние должно быть в пределах start_tolerance от неё — иначе INVALID_GOAL
        # >>> (траектория спланирована из другого состояния: скачок уставки = удар).
        pass  # TODO: проверка отклонения начала траектории
        # <<< STUDENT
        self.traj = traj
        self.names = names
        self.t_start = t_start
        return None

    # ---------------------------------------------------------------- шаг
    def step(self, now, actual):
        """Один период исполнения. actual — словарь «имя -> фактический угол»."""
        t = now - self.t_start
        q_ref, v_ref = self.traj.sample(t)
        q_act = [actual[n] for n in self.names]
        err = [r - a for r, a in zip(q_ref, q_act)]
        # >>> STUDENT ЛР4А-шаг4: контракт исполнителя.
        # >>> 1) t ≤ длительности: превышение допуска по пути любым суставом ->
        # >>>    DONE, PATH_TOLERANCE_VIOLATED (указать сустав и ошибку);
        # >>> 2) t > длительности: все суставы в допуске по цели -> DONE, SUCCESSFUL;
        # >>> 3) t > длительности + допуск по времени, а цель не достигнута ->
        # >>>    DONE, GOAL_TOLERANCE_VIOLATED; иначе RUNNING.
        return Step(RUNNING, None, '', q_ref, v_ref, err)  # TODO: проверка допусков
        # <<< STUDENT

    @staticmethod
    def _fail(code, message):
        return Step(DONE, code, message, None, None, None)
