# -*- coding: utf-8 -*-
"""Траектория как функция времени: проверка и интерполяция (пособие, §4.5.2).

Способ интерполяции выбирается ПО СОДЕРЖИМОМУ сообщения, как в типовом исполнителе:
    только положения                       -> линейная      (C⁰, скачок скорости в узлах)
    положения и скорости                   -> кубическая    (C¹, эрмитова)
    положения, скорости и ускорения        -> квинтическая  (C²)
Отсюда «дёрганое» исполнение гладкого плана, если скорости не переданы.

Модуль не зависит от ROS: вход — списки чисел, время — секунды от начала траектории.
"""
import math


class TrajectoryError(ValueError):
    pass


LINEAR, CUBIC, QUINTIC = 'linear', 'cubic', 'quintic'


def _finite(values):
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


class Trajectory(object):
    """Точки: times[k] (с), positions[k][j], при наличии velocities[k][j], accelerations[k][j].

    Если первая точка задана на момент t > 0, в начало добавляется точка «текущее
    состояние с нулевой скоростью» на момент 0: исполнитель доводит привод до начала
    траектории, а не прыгает в него.
    """

    def __init__(self, times, positions, velocities=None, accelerations=None, start=None):
        n_pts = len(times)
        if n_pts == 0:
            raise TrajectoryError('в траектории нет точек')
        n = len(positions[0]) if positions else 0
        if len(positions) != n_pts or n == 0:
            raise TrajectoryError('число наборов положений не совпадает с числом точек')
        for k in range(n_pts):
            if len(positions[k]) != n or not _finite(positions[k]):
                raise TrajectoryError('точка %d: положения неполные или не числа' % k)
        for k in range(1, n_pts):
            if not times[k] > times[k - 1]:
                raise TrajectoryError('время точек должно строго возрастать (точка %d)' % k)
        if times[0] < 0:
            raise TrajectoryError('отрицательное время первой точки')
        vel = self._optional(velocities, n_pts, n, 'скорости')
        acc = self._optional(accelerations, n_pts, n, 'ускорения')
        if acc is not None and vel is None:
            raise TrajectoryError('ускорения заданы без скоростей')
        self.kind = QUINTIC if acc is not None else (CUBIC if vel is not None else LINEAR)

        self.t = list(times)
        self.q = [list(p) for p in positions]
        self.v = vel
        self.a = acc
        if self.t[0] > 0:
            if start is None:
                raise TrajectoryError('первая точка не в момент 0, а текущее состояние неизвестно')
            if len(start) != n:
                raise TrajectoryError('размер текущего состояния не совпадает с траекторией')
            self.t.insert(0, 0.0)
            self.q.insert(0, list(start))
            if self.v is not None:
                self.v.insert(0, [0.0] * n)
            if self.a is not None:
                self.a.insert(0, [0.0] * n)
        self.n = n
        self.duration = self.t[-1]

    @staticmethod
    def _optional(rows, n_pts, n, what):
        if not rows or all(len(r) == 0 for r in rows):
            return None
        if len(rows) != n_pts or any(len(r) != n or not _finite(r) for r in rows):
            raise TrajectoryError('%s заданы не во всех точках или не для всех суставов' % what)
        return [list(r) for r in rows]

    def sample(self, t):
        """Эталон на момент t: (положения, скорости). До 0 — первая точка, после конца — последняя."""
        if t <= self.t[0]:
            return list(self.q[0]), [0.0] * self.n if self.v is None else list(self.v[0])
        if t >= self.t[-1]:
            return list(self.q[-1]), [0.0] * self.n
        k = 1
        while self.t[k] < t:
            k += 1
        t0, t1 = self.t[k - 1], self.t[k]
        T, tau = t1 - t0, t - t0
        q, qd = [], []
        for j in range(self.n):
            q0, q1 = self.q[k - 1][j], self.q[k][j]
            # >>> STUDENT ЛР4А-шаг3: интерполяция по содержимому сообщения на отрезке [t0; t1]:
            # >>> LINEAR  — q = q0 + (q1 − q0)·τ/T;
            # >>> CUBIC   — эрмитов полином по q0, q1, v0, v1;
            # >>> QUINTIC — полином 5-й степени по q, v, a на обоих концах.
            # >>> Вернуть положение и скорость (производную того же полинома).
            raise NotImplementedError('ЛР4-А, шаг 3: Trajectory.sample')
            # <<< STUDENT
        return q, qd
