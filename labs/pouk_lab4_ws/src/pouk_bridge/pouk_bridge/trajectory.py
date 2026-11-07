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
        """Эталон на момент t: (положения, скорости).

        До начала — первая точка.
        В конце и после конца — последняя точка с нулевой скоростью.
        """
        if t <= self.t[0]:
            velocity = (
                [0.0] * self.n
                if self.v is None
                else list(self.v[0])
            )
            return list(self.q[0]), velocity

        if t >= self.t[-1]:
            return list(self.q[-1]), [0.0] * self.n

        k = 1
        while self.t[k] < t:
            k += 1

        t0, t1 = self.t[k - 1], self.t[k]
        T = t1 - t0
        tau = t - t0

        q, qd = [], []

        for j in range(self.n):
            q0 = self.q[k - 1][j]
            q1 = self.q[k][j]
            h = q1 - q0

            if self.kind == LINEAR:
                position = q0 + h * tau / T
                velocity = h / T

            elif self.kind == CUBIC:
                v0 = self.v[k - 1][j]
                v1 = self.v[k][j]

                c2 = (3.0 * h - (2.0 * v0 + v1) * T) / T**2
                c3 = (-2.0 * h + (v0 + v1) * T) / T**3

                position = (
                    q0
                    + v0 * tau
                    + c2 * tau**2
                    + c3 * tau**3
                )

                velocity = (
                    v0
                    + 2.0 * c2 * tau
                    + 3.0 * c3 * tau**2
                )

            elif self.kind == QUINTIC:
                v0 = self.v[k - 1][j]
                v1 = self.v[k][j]
                a0 = self.a[k - 1][j]
                a1 = self.a[k][j]

                c0 = q0
                c1 = v0
                c2 = a0 / 2.0

                c3 = (
                    20.0 * h
                    - (12.0 * v0 + 8.0 * v1) * T
                    - (3.0 * a0 - a1) * T**2
                ) / (2.0 * T**3)

                c4 = (
                    -30.0 * h
                    + (16.0 * v0 + 14.0 * v1) * T
                    + (3.0 * a0 - 2.0 * a1) * T**2
                ) / (2.0 * T**4)

                c5 = (
                    12.0 * h
                    - 6.0 * (v0 + v1) * T
                    - (a0 - a1) * T**2
                ) / (2.0 * T**5)

                position = (
                    c0
                    + c1 * tau
                    + c2 * tau**2
                    + c3 * tau**3
                    + c4 * tau**4
                    + c5 * tau**5
                )

                velocity = (
                    c1
                    + 2.0 * c2 * tau
                    + 3.0 * c3 * tau**2
                    + 4.0 * c4 * tau**3
                    + 5.0 * c5 * tau**4
                )

            else:
                raise TrajectoryError(
                    'неизвестный способ интерполяции: %s' % self.kind
                )

            q.append(position)
            qd.append(velocity)

        return q, qd
